// src/drivers/imu/calcul_calibration.rs
//! Calculs des calibrations de la centrale inertielle — fonctions pures
//!
//! Procédures décrites dans `doc/calibration.md`. Ce module ne fait que les
//! calculs, sans accès matériel ni console : il travaille sur des
//! [`EchantillonCapteur`] (repère de la carte) et se teste sur données
//! simulées. Le déroulé interactif est dans `diagnostiques/diag_mpu9250.rs`.
//!
//! # Repères
//!
//! - **Repère capteur** : axes de l'accéléromètre du MPU9250 (axes AK8963
//!   déjà réalignés). L'accéléromètre y mesure la force spécifique : +g sur
//!   l'axe tourné vers le haut au repos.
//! - **Repère avion** : X nez, Y aile droite, Z vers le bas (convention de
//!   `estimation/attitude.rs`).
//!
//! La rotation de montage `M` (3×3, lignes = axes avion exprimés dans le
//! repère capteur) fait passer de l'un à l'autre : `v_avion = M · v_capteur`.

use crate::types::{Result, ErreursAirHaum, Vector3};

/// Gravité standard (m/s²).
pub const G: f32 = 9.80665;

/// Rotation 3×3, lignes = axes du repère d'arrivée exprimés dans le repère
/// de départ.
pub type Matrice3 = [[f32; 3]; 3];

/// Montage par défaut, sans calibration : carte à plat, X vers le nez,
/// composants vers le haut (repère capteur X avant, Y gauche, Z haut).
pub const MONTAGE_DEFAUT: Matrice3 = [
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
];

/// Seuil d'immobilité gyroscope : écart maximal à la moyenne (rad/s, ≈ 3°/s).
pub const SEUIL_IMMOBILE_GYRO: f32 = 0.05;
/// Seuil d'immobilité accéléromètre : écart maximal à la moyenne (m/s²).
/// Assez large pour un empennage tenu à la main.
pub const SEUIL_IMMOBILE_ACCEL: f32 = 0.5;
/// Variation de tangage minimale entre les deux positions de l'atelier.
pub const ECART_MONTAGE_MIN_DEG: f32 = 3.0;
/// Tolérance de la vérification de montage sur le terrain (pente du sol).
pub const TOLERANCE_VERIFICATION_DEG: f32 = 3.0;
/// Dispersion maximale des points autour du cercle, en fraction du rayon.
pub const DISPERSION_CERCLE_MAX: f32 = 0.10;
/// Écart maximal toléré entre composante horizontale mesurée et attendue.
pub const ECART_CHAMP_HORIZONTAL_MAX: f32 = 0.25;
/// Inclinaison magnétique par défaut (degrés), utilisée quand le terrain
/// n'est pas renseigné : le Mans, WMM2025 en 2026. Une erreur de 1° donne
/// environ 2 µT sur le décalage vertical (`doc/calibration.md`).
pub const INCLINAISON_DEFAUT_DEG: f32 = 63.5;
/// Intensité du champ par défaut (µT) : le Mans, WMM2025 en 2026.
pub const CHAMP_DEFAUT_UT: f32 = 48.1;

/// Une lecture de la carte dans le repère capteur, sans calibration.
#[derive(Debug, Clone, Copy)]
pub struct EchantillonCapteur {
    /// Force spécifique (m/s²).
    pub accel: Vector3,
    /// Vitesse angulaire (rad/s).
    pub gyro: Vector3,
    /// Champ magnétique (µT, ASA appliqués), si une nouvelle mesure était prête.
    pub mag: Option<Vector3>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Outils
// ─────────────────────────────────────────────────────────────────────────────

pub fn appliquer_matrice(m: &Matrice3, v: Vector3) -> Vector3 {
    Vector3::nouveau(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

pub fn moyenne(vs: impl Iterator<Item = Vector3>) -> Vector3 {
    let mut somme = Vector3::zero();
    let mut n = 0usize;
    for v in vs {
        somme = somme + v;
        n += 1;
    }
    if n == 0 { Vector3::zero() } else { somme / n as f32 }
}

fn ecart_max(vs: impl Iterator<Item = Vector3> + Clone) -> f32 {
    let m = moyenne(vs.clone());
    vs.map(|v| (v - m).norme()).fold(0.0, f32::max)
}

/// Vrai si la fenêtre d'échantillons correspond à un appareil immobile.
pub fn est_immobile(echantillons: &[EchantillonCapteur]) -> bool {
    !echantillons.is_empty()
        && ecart_max(echantillons.iter().map(|e| e.gyro))  < SEUIL_IMMOBILE_GYRO
        && ecart_max(echantillons.iter().map(|e| e.accel)) < SEUIL_IMMOBILE_ACCEL
}

/// Retire le décalage de Z d'une force spécifique (repère capteur).
pub fn corriger_z(f: Vector3, decalage_z: f32) -> Vector3 {
    Vector3::nouveau(f.x, f.y, f.z - decalage_z)
}

/// Gravité apparente dans le repère avion (convention de l'estimation),
/// à partir d'une force spécifique capteur déjà corrigée de Z.
pub fn gravite_apparente_avion(montage: &Matrice3, f: Vector3) -> Vector3 {
    appliquer_matrice(montage, f) * -1.0
}

/// Roulis et tangage (degrés) d'une gravité apparente dans le repère avion,
/// mêmes formules que `estimation/attitude.rs`.
pub fn attitude_depuis_gravite(a: Vector3) -> (f32, f32) {
    let roulis  = a.y.atan2(a.z);
    let tangage = (-a.x).atan2((a.y * a.y + a.z * a.z).sqrt());
    (roulis.to_degrees(), tangage.to_degrees())
}

// ─────────────────────────────────────────────────────────────────────────────
// Accéléromètre
// ─────────────────────────────────────────────────────────────────────────────

/// Décalage de Z ramenant la norme d'une force spécifique moyenne au repos
/// à 1 g, X et Y inchangés. `None` si X et Y dépassent déjà 1 g.
///
/// Deux racines possibles : on retient la plus petite en valeur absolue.
pub fn decalage_z(f_repos: Vector3) -> Option<f32> {
    let h2 = G * G - f_repos.x * f_repos.x - f_repos.y * f_repos.y;
    if h2 <= 0.0 {
        return None;
    }
    let h = h2.sqrt();
    let (a, b) = (f_repos.z - h, f_repos.z + h);
    Some(if a.abs() <= b.abs() { a } else { b })
}

/// Rotation de montage à partir des deux positions de l'atelier (forces
/// spécifiques moyennes, repère capteur, corrigées de Z) :
/// - `f_sol` : queue au sol (nez haut) ;
/// - `f_horizontal` : fuselage horizontal.
///
/// Z avion = opposé de la verticale du fuselage horizontal ; Y avion = axe
/// de la rotation entre les deux positions (passer de nez haut à
/// l'horizontale fait tourner la gravité autour de l'aile droite) ;
/// X = Y × Z.
pub fn rotation_montage(f_sol: Vector3, f_horizontal: Vector3) -> Result<Matrice3> {
    let haut = f_horizontal.normalise();
    let axe  = f_sol.normalise().cross(&haut);
    let ecart_deg = axe.norme().asin().to_degrees();
    if ecart_deg < ECART_MONTAGE_MIN_DEG {
        return Err(ErreursAirHaum::CalibrationEchouee(format!(
            "écart de tangage entre les deux positions trop faible : {:.1}° (minimum {:.0}°)",
            ecart_deg, ECART_MONTAGE_MIN_DEG
        )));
    }

    let z = haut * -1.0;
    let y = (axe - z * axe.dot(&z)).normalise();
    let x = y.cross(&z);
    Ok([[x.x, x.y, x.z], [y.x, y.y, y.z], [z.x, z.y, z.z]])
}

/// Écart de tangage (degrés) entre deux forces spécifiques.
pub fn ecart_angulaire_deg(a: Vector3, b: Vector3) -> f32 {
    a.normalise().dot(&b.normalise()).clamp(-1.0, 1.0).acos().to_degrees()
}

/// Nom de l'axe de la carte le plus proche d'une direction (« +X », « −Z »…),
/// pour l'affichage du montage détecté.
pub fn axe_carte_dominant(direction: [f32; 3]) -> String {
    let (i, v) = direction.iter().enumerate()
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .map(|(i, v)| (i, *v))
        .unwrap_or((0, 0.0));
    format!("{}{}", if v >= 0.0 { "+" } else { "−" }, ["X", "Y", "Z"][i])
}

// ─────────────────────────────────────────────────────────────────────────────
// Magnétomètre — tour au sol
// ─────────────────────────────────────────────────────────────────────────────
//
// Avec une roue de queue, l'avion ne tourne pas sans lever l'empennage : la
// verticale vue par la carte varie d'une dizaine de degrés pendant le tour.
// Elle est donc suivie à chaque instant (gyroscope, recalé lentement sur
// l'accéléromètre), et le modèle n'utilise que cette verticale, pas le cap
// (insensible à l'erreur d'échelle du gyroscope).

/// Constante de temps du recalage de la verticale sur l'accéléromètre (s) :
/// assez longue pour lisser les accélérations dues à la poussée de l'avion,
/// assez courte pour effacer la dérive du gyroscope.
pub const CONSTANTE_RECALAGE_VERTICALE_S: f32 = 2.0;

/// Angle parcouru (degrés, signé) autour de la verticale `haut` (unitaire,
/// repère capteur), par intégration du gyroscope corrigé de son biais.
pub fn angle_autour(haut: Vector3, gyro: Vector3, biais: Vector3, dt_s: f32) -> f32 {
    (gyro - biais).dot(&haut) * dt_s * 180.0 / std::f32::consts::PI
}

/// Propage la verticale `haut` (unitaire, repère capteur) sur `dt_s` : un
/// vecteur fixe dans le monde tourne en −ω vu de la carte ; recalage lent
/// sur la force spécifique `f_corrigee` (corrigée de Z).
pub fn suivre_verticale(haut: Vector3, gyro: Vector3, biais: Vector3, f_corrigee: Vector3, dt_s: f32) -> Vector3 {
    let propagee = haut + haut.cross(&(gyro - biais)) * dt_s;
    let gain = (dt_s / CONSTANTE_RECALAGE_VERTICALE_S).min(1.0);
    (propagee + (f_corrigee.normalise() - propagee) * gain).normalise()
}

/// Une mesure du tour : champ et verticale au même instant (repère capteur).
#[derive(Debug, Clone, Copy)]
pub struct MesureTour {
    pub mag:  Vector3,
    pub haut: Vector3,
}

/// Résultat de l'ajustement du tour au sol.
#[derive(Debug, Clone, Copy)]
pub struct ResultatTourMag {
    /// Décalage dû à l'avion (µT, repère capteur).
    pub decalage: Vector3,
    /// Composante horizontale du champ mesurée (µT).
    pub champ_horizontal_ut: f32,
    /// Dispersion RMS de la composante horizontale autour du modèle (µT).
    pub dispersion_ut: f32,
    /// Dispersion RMS de la composante verticale autour du modèle (µT).
    pub dispersion_verticale_ut: f32,
}

/// Ajuste le décalage `b` et la composante horizontale `Bh` du champ sur un
/// tour au sol. Pour chaque mesure, avec `h` la verticale du moment :
///
/// ```text
/// horizontale : |(m − b) − h·((m − b)·h)| = Bh
/// verticale   :  (m − b)·h = −Bh·tan(I)      (I > 0 : le champ plonge vers le sol)
/// ```
///
/// Le décalage vertical n'est mesurable que par l'inclinaison magnétique du
/// terrain (les variations d'inclinaison de l'avion, ~10°, sont trop faibles).
/// Départ : cercle algébrique (Kåsa) autour de la verticale moyenne, puis
/// Gauss-Newton sur (b, Bh).
pub fn ajuster_tour_mag(mesures: &[MesureTour], inclinaison_deg: f32) -> Result<ResultatTourMag> {
    if mesures.len() < 20 {
        return Err(ErreursAirHaum::CalibrationEchouee(format!(
            "trop peu de mesures magnétomètre : {}", mesures.len()
        )));
    }
    let tan_i = (inclinaison_deg as f64).to_radians().tan();

    // ── Départ : cercle autour de la verticale moyenne ──────────────────────
    let haut = moyenne(mesures.iter().map(|m| m.haut)).normalise();
    let auxiliaire = if haut.x.abs() < 0.9 { Vector3::nouveau(1.0, 0.0, 0.0) }
                     else { Vector3::nouveau(0.0, 1.0, 0.0) };
    let e1 = (auxiliaire - haut * auxiliaire.dot(&haut)).normalise();
    let e2 = haut.cross(&e1);

    // x² + y² + D·x + E·y + F = 0 par moindres carrés
    let mut a = [[0.0f64; 3]; 3];
    let mut b = [0.0f64; 3];
    for m in mesures {
        let (x, y) = (m.mag.dot(&e1) as f64, m.mag.dot(&e2) as f64);
        let ligne = [x, y, 1.0];
        let cible = -(x * x + y * y);
        for i in 0..3 {
            for j in 0..3 { a[i][j] += ligne[i] * ligne[j]; }
            b[i] += ligne[i] * cible;
        }
    }
    let [d, e, f] = resoudre(a, b).ok_or_else(|| ErreursAirHaum::CalibrationEchouee(
        "ajustement du cercle impossible (mesures alignées ?)".into()
    ))?;
    let (cx, cy) = (-d / 2.0, -e / 2.0);
    let r2 = cx * cx + cy * cy - f;
    if r2 <= 0.0 {
        return Err(ErreursAirHaum::CalibrationEchouee("ajustement du cercle dégénéré".into()));
    }
    let rayon = r2.sqrt();
    let vertical_moyen = mesures.iter().map(|m| m.mag.dot(&haut) as f64).sum::<f64>() / mesures.len() as f64;
    let depart = e1 * cx as f32 + e2 * cy as f32 + haut * (vertical_moyen + rayon * tan_i) as f32;
    let mut p = [depart.x as f64, depart.y as f64, depart.z as f64, rayon];

    // ── Gauss-Newton sur (b, Bh) ────────────────────────────────────────────
    let residus = |p: &[f64; 4], m: &MesureTour| -> [f64; 2] {
        let (mx, my, mz) = (m.mag.x as f64 - p[0], m.mag.y as f64 - p[1], m.mag.z as f64 - p[2]);
        let (hx, hy, hz) = (m.haut.x as f64, m.haut.y as f64, m.haut.z as f64);
        let v = mx * hx + my * hy + mz * hz;
        let horizontal = (mx * mx + my * my + mz * mz - v * v).max(0.0).sqrt();
        [horizontal - p[3], v + p[3] * tan_i]
    };
    for _ in 0..20 {
        let mut jtj = [[0.0f64; 4]; 4];
        let mut jtr = [0.0f64; 4];
        for m in mesures {
            let r0 = residus(&p, m);
            let mut jac = [[0.0f64; 4]; 2];
            for k in 0..4 {
                let mut pk = p;
                pk[k] += 1e-4;
                let rk = residus(&pk, m);
                for l in 0..2 { jac[l][k] = (rk[l] - r0[l]) / 1e-4; }
            }
            for l in 0..2 {
                for i in 0..4 {
                    jtr[i] -= jac[l][i] * r0[l];
                    for j in 0..4 { jtj[i][j] += jac[l][i] * jac[l][j]; }
                }
            }
        }
        let pas = resoudre(jtj, jtr).ok_or_else(|| ErreursAirHaum::CalibrationEchouee(
            "ajustement du tour impossible (système singulier)".into()
        ))?;
        for k in 0..4 { p[k] += pas[k]; }
        if pas.iter().map(|x| x * x).sum::<f64>().sqrt() < 1e-5 { break; }
    }
    if p[3] <= 0.0 {
        return Err(ErreursAirHaum::CalibrationEchouee("ajustement du tour dégénéré".into()));
    }

    let n = mesures.len() as f64;
    let (mut s_h, mut s_v) = (0.0f64, 0.0f64);
    for m in mesures {
        let [rh, rv] = residus(&p, m);
        s_h += rh * rh;
        s_v += rv * rv;
    }
    Ok(ResultatTourMag {
        decalage: Vector3::nouveau(p[0] as f32, p[1] as f32, p[2] as f32),
        champ_horizontal_ut: p[3] as f32,
        dispersion_ut: (s_h / n).sqrt() as f32,
        dispersion_verticale_ut: (s_v / n).sqrt() as f32,
    })
}

/// Vérifie la qualité d'un tour : dispersion et cohérence avec le terrain.
/// Retourne la liste des défauts (vide si accepté).
pub fn defauts_tour_mag(r: &ResultatTourMag, champ_total_ut: f32, inclinaison_deg: f32) -> Vec<String> {
    let mut defauts = Vec::new();
    if r.dispersion_ut > DISPERSION_CERCLE_MAX * r.champ_horizontal_ut {
        defauts.push(format!(
            "composante horizontale trop dispersée : {:.1} µT pour {:.1} µT",
            r.dispersion_ut, r.champ_horizontal_ut
        ));
    }
    let vertical = r.champ_horizontal_ut * inclinaison_deg.to_radians().tan();
    if r.dispersion_verticale_ut > DISPERSION_CERCLE_MAX * vertical {
        defauts.push(format!(
            "composante verticale trop dispersée : {:.1} µT pour {:.1} µT",
            r.dispersion_verticale_ut, vertical
        ));
    }
    let attendu = champ_total_ut * inclinaison_deg.to_radians().cos();
    if (r.champ_horizontal_ut - attendu).abs() > ECART_CHAMP_HORIZONTAL_MAX * attendu {
        defauts.push(format!(
            "composante horizontale {:.1} µT, attendue {:.1} µT au terrain",
            r.champ_horizontal_ut, attendu
        ));
    }
    defauts
}

/// Résolution d'un système N×N par élimination de Gauss avec pivot partiel.
fn resoudre<const N: usize>(mut a: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
    for col in 0..N {
        let pivot = (col..N).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-9 { return None; }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for ligne in col + 1..N {
            let k = a[ligne][col] / a[col][col];
            for j in col..N { a[ligne][j] -= k * a[col][j]; }
            b[ligne] -= k * b[col];
        }
    }
    let mut x = [0.0f64; N];
    for i in (0..N).rev() {
        let somme: f64 = (i + 1..N).map(|j| a[i][j] * x[j]).sum();
        x[i] = (b[i] - somme) / a[i][i];
    }
    Some(x)
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn proche(a: f32, b: f32, tol: f32) -> bool { (a - b).abs() < tol }

    /// Force spécifique vue par une carte montée « X nez, Y gauche, Z haut »
    /// (repère capteur = FLU), avion au tangage `tangage_deg` (nez haut > 0).
    fn f_flu(tangage_deg: f32) -> Vector3 {
        let t = tangage_deg.to_radians();
        Vector3::nouveau(G * t.sin(), 0.0, G * t.cos())
    }

    /// Rotation autour de Z capteur, pour simuler une carte tournée dans l'avion.
    fn tourner_z(v: Vector3, deg: f32) -> Vector3 {
        let (s, c) = deg.to_radians().sin_cos();
        Vector3::nouveau(c * v.x - s * v.y, s * v.x + c * v.y, v.z)
    }

    #[test]
    fn decalage_z_ramene_la_norme_a_1g() {
        let f = Vector3::nouveau(0.38, 0.81, 14.17);
        let oz = decalage_z(f).unwrap();
        assert!(proche(corriger_z(f, oz).norme(), G, 1e-3));
        assert!(proche(oz, 4.39, 0.02), "oz = {}", oz);
    }

    #[test]
    fn decalage_z_carte_retournee() {
        let oz = decalage_z(Vector3::nouveau(0.0, 0.0, -9.0)).unwrap();
        assert!(proche(oz, 0.80665, 1e-3), "oz = {}", oz);
    }

    #[test]
    fn decalage_z_impossible_si_xy_trop_grands() {
        assert!(decalage_z(Vector3::nouveau(8.0, 8.0, 0.0)).is_none());
    }

    #[test]
    fn montage_carte_x_avant_donne_le_montage_par_defaut() {
        let m = rotation_montage(f_flu(10.0), f_flu(0.0)).unwrap();
        for i in 0..3 { for j in 0..3 {
            assert!(proche(m[i][j], MONTAGE_DEFAUT[i][j], 1e-5), "m = {:?}", m);
        }}
    }

    #[test]
    fn montage_attitude_au_sol_nez_haut_positif() {
        let m = rotation_montage(f_flu(10.0), f_flu(0.0)).unwrap();
        let (roulis, tangage) = attitude_depuis_gravite(gravite_apparente_avion(&m, f_flu(10.0)));
        assert!(proche(tangage, 10.0, 0.01), "tangage = {}", tangage);
        assert!(proche(roulis, 0.0, 0.01));
        let (_, tangage_h) = attitude_depuis_gravite(gravite_apparente_avion(&m, f_flu(0.0)));
        assert!(proche(tangage_h, 0.0, 0.01));
    }

    #[test]
    fn montage_carte_tournee_de_90_degres() {
        // Carte tournée d'un quart de tour à plat : son +Y pointe vers le nez
        let (sol, hor) = (tourner_z(f_flu(10.0), 90.0), tourner_z(f_flu(0.0), 90.0));
        let m = rotation_montage(sol, hor).unwrap();
        assert_eq!(axe_carte_dominant(m[0]), "+Y");
        let (_, tangage) = attitude_depuis_gravite(gravite_apparente_avion(&m, sol));
        assert!(proche(tangage, 10.0, 0.01), "tangage = {}", tangage);
    }

    #[test]
    fn montage_conserve_le_sens_des_rotations() {
        // Une rotation nez vers la droite (lacet +) autour du Z avion (bas)
        // est vue par une carte FLU comme une rotation autour de −Z capteur.
        let m = rotation_montage(f_flu(10.0), f_flu(0.0)).unwrap();
        let w = appliquer_matrice(&m, Vector3::nouveau(0.0, 0.0, -1.0));
        assert!(proche(w.z, 1.0, 1e-5));
        // Rotation aile droite en bas (roulis +) = autour de +X capteur
        let w = appliquer_matrice(&m, Vector3::nouveau(1.0, 0.0, 0.0));
        assert!(proche(w.x, 1.0, 1e-5));
    }

    #[test]
    fn montage_refuse_si_ecart_trop_faible() {
        assert!(rotation_montage(f_flu(1.0), f_flu(0.0)).is_err());
    }

    #[test]
    fn immobilite_detectee() {
        let fixe = EchantillonCapteur { accel: f_flu(0.0), gyro: Vector3::zero(), mag: None };
        let mut serie = vec![fixe; 50];
        assert!(est_immobile(&serie));
        serie[10].gyro = Vector3::nouveau(0.0, 0.0, 0.3);
        assert!(!est_immobile(&serie));
    }

    fn rot_x(v: Vector3, deg: f32) -> Vector3 {
        let (s, c) = deg.to_radians().sin_cos();
        Vector3::nouveau(v.x, c * v.y - s * v.z, s * v.y + c * v.z)
    }

    /// Mesure simulée : avion au cap `cap_deg`, empennage levé de
    /// `levee_deg` (rotation autour de X capteur), champ terrestre
    /// (`inclinaison_deg`, `total` µT) plus un décalage `b`.
    fn mesure_tour(cap_deg: f32, levee_deg: f32, b: Vector3, inclinaison_deg: f32, total: f32) -> MesureTour {
        let (bh, bv) = (total * inclinaison_deg.to_radians().cos(), total * inclinaison_deg.to_radians().sin());
        let champ_monde = Vector3::nouveau(bh, 0.0, -bv);          // Z monde vers le haut
        let vers_capteur = |v: Vector3| rot_x(tourner_z(v, -cap_deg), levee_deg);
        MesureTour {
            mag:  vers_capteur(champ_monde) + b,
            haut: vers_capteur(Vector3::nouveau(0.0, 0.0, 1.0)),
        }
    }

    #[test]
    fn tour_mag_retrouve_le_decalage() {
        let b = Vector3::nouveau(-63.9, 95.5, -74.8);
        let mesures: Vec<MesureTour> = (0..360).step_by(3)
            .map(|cap| mesure_tour(cap as f32, 0.0, b, 63.0, 48.0)).collect();
        let r = ajuster_tour_mag(&mesures, 63.0).unwrap();
        assert!((r.decalage - b).norme() < 0.05, "décalage = {:?}", r.decalage);
        assert!(proche(r.champ_horizontal_ut, 48.0 * 63_f32.to_radians().cos(), 0.05));
        assert!(defauts_tour_mag(&r, 48.0, 63.0).is_empty());
    }

    #[test]
    fn tour_mag_empennage_leve_pendant_le_tour() {
        // Empennage levé jusqu'à 12° selon le cap, comme avec une roue de queue
        let b = Vector3::nouveau(49.1, -61.8, 123.1);
        let mesures: Vec<MesureTour> = (0..360).step_by(2)
            .map(|cap| mesure_tour(cap as f32, 12.0 * (cap as f32 * 0.035).sin().abs(), b, 63.0, 48.0))
            .collect();
        let r = ajuster_tour_mag(&mesures, 63.0).unwrap();
        assert!((r.decalage - b).norme() < 0.05, "décalage = {:?}", r.decalage);
        assert!(r.dispersion_ut < 0.01 && r.dispersion_verticale_ut < 0.01, "{:?}", r);
        assert!(defauts_tour_mag(&r, 48.0, 63.0).is_empty());
    }

    #[test]
    fn tour_mag_refuse_si_terrain_incoherent() {
        let mesures: Vec<MesureTour> = (0..360).step_by(5)
            .map(|cap| mesure_tour(cap as f32, 0.0, Vector3::zero(), 63.0, 48.0)).collect();
        let r = ajuster_tour_mag(&mesures, 63.0).unwrap();
        // Terrain déclaré avec un champ deux fois plus fort
        assert_eq!(defauts_tour_mag(&r, 96.0, 63.0).len(), 1);
    }

    #[test]
    fn verticale_suivie_pendant_une_levee_d_empennage() {
        // Rotation de 10° en 2 s autour de X capteur, accéléromètre cohérent
        let biais = Vector3::zero();
        let w = Vector3::nouveau(-(10_f32.to_radians()) / 2.0, 0.0, 0.0);
        let mut haut = Vector3::nouveau(0.0, 0.0, 1.0);
        for i in 1..=200 {
            let vraie = rot_x(Vector3::nouveau(0.0, 0.0, 1.0), 10.0 * i as f32 / 200.0);
            haut = suivre_verticale(haut, w, biais, vraie * G, 0.01);
        }
        let attendue = rot_x(Vector3::nouveau(0.0, 0.0, 1.0), 10.0);
        assert!(ecart_angulaire_deg(haut, attendue) < 0.1, "écart {:.3}°", ecart_angulaire_deg(haut, attendue));
    }

    #[test]
    fn angle_autour_integre_un_tour() {
        let haut = Vector3::nouveau(0.0, 0.0, 1.0);
        let biais = Vector3::nouveau(0.01, 0.0, 0.02);
        let w = haut * (2.0 * std::f32::consts::PI / 60.0) + biais; // un tour en 60 s
        let total: f32 = (0..6000).map(|_| angle_autour(haut, w, biais, 0.01)).sum();
        assert!(proche(total, 360.0, 0.1), "total = {}", total);
    }
}

