// src/estimation/ekf_navigation.rs
//! Filtre de Kalman Étendu global (navigation + attitude).
//!
//! Vecteur d'état x ∈ ℝ¹³ :
//! ```text
//! [0..3]   position NED   (m, relative à l'origine = premier fix GPS)
//! [3..6]   vitesse NED    (m/s)
//! [6..10]  quaternion     [qw, qx, qy, qz]
//! [10..13] biais gyro     (rad/s)
//! ```
//!
//! # Sources de mesure
//!
//! | Source        | Fréquence | Observations        |
//! |---------------|-----------|---------------------|
//! | Accéléromètre | ~200 Hz   | gravité corps = f(q)|
//! | GPS position  |  ~10 Hz   | p_n, p_e, p_d       |
//! | GPS vitesse   |  ~10 Hz   | v_n, v_e, v_d       |
//!
//! # Coordonnées NED
//!
//! Le zéro NED est fixé au premier fix GPS valide reçu par `corriger_gps`.
//! L'approximation Terre-plate est valide pour des distances < 20 km.

use crate::estimation::attitude::Attitude;
use crate::types::{DonneesGps, Horodatage, Quaternion, Vector3};

// ─────────────────────────────────────────────────────────────────────────────
// Constantes
// ─────────────────────────────────────────────────────────────────────────────

const G: f32 = 9.80665;
const R_TERRE: f64 = 6_371_000.0;

/// Variance bruit processus — attitude quaternion (rad²/s · s).
const Q_ATT:   f32 = 1e-4;
/// Variance bruit processus — biais gyroscope (rad²/s² · s).
const Q_BIAIS: f32 = 1e-6;
/// Variance bruit processus — vitesse via accéléromètre ((m/s²)² · s).
const Q_VIT:   f32 = 0.01;

/// Variance bruit mesure accéléromètre ((m/s²)²).
const R_ACCEL: f32 = 0.09;
/// Plancher de variance GPS position (m²) — utilisé si hAcc trop optimiste.
const R_GPS_POS_PLANCHER: f32 = 4.0;   // équivaut à σ = 2 m
/// Variance bruit mesure GPS vitesse ((m/s)²).
const R_GPS_VIT: f32 = 0.25;           // équivaut à σ = 0.5 m/s

/// Seuil de rejet accéléromètre : |‖a‖ − G| > seuil → pas de correction.
const SEUIL_ACCEL: f32 = 0.3;

// ─────────────────────────────────────────────────────────────────────────────
// Alias de types
// ─────────────────────────────────────────────────────────────────────────────

type V13   = [f32; 13];
type M13   = [[f32; 13]; 13];
type M3x13 = [[f32; 13]; 3];
type M13x3 = [[f32; 3]; 13];
type M33   = [[f32; 3]; 3];
type V3    = [f32; 3];
type M44   = [[f32; 4]; 4];
type M34   = [[f32; 4]; 3];
type M43   = [[f32; 3]; 4];

// ─────────────────────────────────────────────────────────────────────────────
// Origine NED
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy)]
struct OrigineNed {
    lat: f64,
    lon: f64,
    alt: f32,
}

// ─────────────────────────────────────────────────────────────────────────────
// Type de sortie
// ─────────────────────────────────────────────────────────────────────────────

/// État de navigation complet estimé par l'EKF global.
#[derive(Debug, Clone, Copy)]
pub struct EtatNavigation {
    pub horodatage:      Horodatage,
    pub attitude:        Attitude,
    /// Position NED relative à l'origine (m). Nul avant le premier fix GPS.
    pub position_ned:    Vector3,
    /// Vitesse NED (m/s).
    pub vitesse_ned:     Vector3,
    /// Biais gyroscope estimé (rad/s).
    pub biais_gyro:      Vector3,
    /// Vrai dès qu'un premier fix GPS valide a été intégré.
    pub origine_definie: bool,
}

impl EtatNavigation {
    pub fn nul() -> Self {
        Self {
            horodatage:      Horodatage::maintenant(),
            attitude:        Attitude::nulle(),
            position_ned:    Vector3::zero(),
            vitesse_ned:     Vector3::zero(),
            biais_gyro:      Vector3::zero(),
            origine_definie: false,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Structure principale
// ─────────────────────────────────────────────────────────────────────────────

/// Filtre de Kalman Étendu global — navigation + attitude, 13 états.
pub struct EkfNavigation {
    x:          V13,
    p:          M13,
    origine:    Option<OrigineNed>,
    initialise: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// API publique
// ─────────────────────────────────────────────────────────────────────────────

impl EkfNavigation {
    pub fn nouveau() -> Self {
        let mut x = [0.0f32; 13];
        x[6] = 1.0;  // quaternion identité : qw = 1
        let mut p = [[0.0f32; 13]; 13];
        for i in 0..3   { p[i][i] = 1e6;  }   // position : inconnue avant le 1er fix (~1 km σ)
        for i in 3..6   { p[i][i] = 100.0; }  // vitesse  : inconnue au démarrage (~10 m/s σ)
        for i in 6..10  { p[i][i] = 1.0;  }   // quaternion
        for i in 10..13 { p[i][i] = 1e-2; }   // biais gyro
        Self { x, p, origine: None, initialise: false }
    }

    /// Étape de prédiction — à appeler à chaque mesure IMU (~200 Hz).
    ///
    /// La première mesure initialise l'attitude depuis l'accéléromètre.
    pub fn predire(&mut self, a_meas: Vector3, omega_meas: Vector3, dt: f32) {
        if !self.initialise {
            self.initialiser_depuis_accel(a_meas);
            return;
        }
        if dt <= 0.0 { return; }

        let bg    = Vector3::nouveau(self.x[10], self.x[11], self.x[12]);
        let omega = omega_meas - bg;
        let q     = self.quat();

        // Accélération NED = R(q)·a_corps − g
        let a_ned = corps_vers_monde(&q, a_meas)
                  - Vector3::nouveau(0.0, 0.0, G);

        // ── Propagation état ─────────────────────────────────────────────────
        let dt2h = 0.5 * dt * dt;
        self.x[0] += self.x[3] * dt + a_ned.x * dt2h;
        self.x[1] += self.x[4] * dt + a_ned.y * dt2h;
        self.x[2] += self.x[5] * dt + a_ned.z * dt2h;
        self.x[3] += a_ned.x * dt;
        self.x[4] += a_ned.y * dt;
        self.x[5] += a_ned.z * dt;
        let q_new = q.multiplier(&Quaternion::depuis_gyroscope(omega, dt)).normalise();
        self.x[6] = q_new.w;
        self.x[7] = q_new.x;
        self.x[8] = q_new.y;
        self.x[9] = q_new.z;
        // biais gyro : marche aléatoire, inchangé

        // ── Propagation covariance  P ← F·P·Fᵀ + Q ──────────────────────────
        let f    = construire_f(&q, omega, a_meas, dt);
        let fpft = mult_m13(&mult_m13(&f, &self.p), &transposer_m13(&f));
        self.p   = ajouter_m13(&fpft, &bruit_processus(dt));
    }

    /// Correction GPS : position + vitesse NED.
    ///
    /// À appeler uniquement si `gps.type_fix.est_valide()`.
    pub fn corriger_gps(&mut self, gps: &DonneesGps) {
        if !self.initialise { return; }

        if self.origine.is_none() {
            self.origine = Some(OrigineNed {
                lat: gps.latitude,
                lon: gps.longitude,
                alt: gps.altitude_msl,
            });
        }
        let orig = *self.origine.as_ref().unwrap();

        // ── Position ─────────────────────────────────────────────────────────
        let pos   = lat_lon_alt_vers_ned(gps.latitude, gps.longitude, gps.altitude_msl, &orig);
        let r_pos = (gps.precision_h * gps.precision_h).max(R_GPS_POS_PLANCHER);
        let y_pos: V3 = [pos.x - self.x[0], pos.y - self.x[1], pos.z - self.x[2]];
        self.corriger_pos(y_pos, r_pos);

        // ── Vitesse ──────────────────────────────────────────────────────────
        let y_vel: V3 = [
            gps.vel_nord - self.x[3],
            gps.vel_est  - self.x[4],
            gps.vel_bas  - self.x[5],
        ];
        self.corriger_vel(y_vel, R_GPS_VIT);
    }

    /// Correction accéléromètre (direction gravité dans le repère corps).
    ///
    /// À appeler à chaque mesure IMU, après `predire`.
    pub fn corriger_accel(&mut self, a_meas: Vector3) {
        if !self.initialise { return; }
        if (a_meas.norme() - G).abs() > SEUIL_ACCEL { return; }

        let q     = self.quat();
        let g_att = q.monde_vers_corps(Vector3::nouveau(0.0, 0.0, G));
        let y: V3 = [a_meas.x - g_att.x, a_meas.y - g_att.y, a_meas.z - g_att.z];
        let h     = h_accel(&q);
        self.corriger_generique(y, h, R_ACCEL);
    }

    /// Retourne l'état de navigation courant.
    pub fn etat(&self) -> EtatNavigation {
        let q = self.quat();
        let (roulis, tangage, lacet) = q.vers_euler();
        EtatNavigation {
            horodatage:      Horodatage::maintenant(),
            attitude:        Attitude { roulis, tangage, lacet },
            position_ned:    Vector3::nouveau(self.x[0], self.x[1], self.x[2]),
            vitesse_ned:     Vector3::nouveau(self.x[3], self.x[4], self.x[5]),
            biais_gyro:      Vector3::nouveau(self.x[10], self.x[11], self.x[12]),
            origine_definie: self.origine.is_some(),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Implémentation interne
// ─────────────────────────────────────────────────────────────────────────────

impl EkfNavigation {
    fn initialiser_depuis_accel(&mut self, a: Vector3) {
        let roulis  = a.y.atan2(a.z);
        let tangage = (-a.x).atan2((a.y * a.y + a.z * a.z).sqrt());
        let cr = (roulis  / 2.0).cos(); let sr = (roulis  / 2.0).sin();
        let cp = (tangage / 2.0).cos(); let sp = (tangage / 2.0).sin();
        let q = Quaternion::nouveau(cp*cr, cp*sr, sp*cr, -sp*sr).normalise();
        self.x[6] = q.w; self.x[7] = q.x; self.x[8] = q.y; self.x[9] = q.z;
        self.initialise = true;
    }

    fn quat(&self) -> Quaternion {
        Quaternion::nouveau(self.x[6], self.x[7], self.x[8], self.x[9])
    }

    /// Correction avec H_pos = [I₃ | 0 | 0 | 0] — structure creuse exploitée.
    fn corriger_pos(&mut self, y: V3, r_var: f32) {
        // S = P[0:3, 0:3] + r·I₃
        let mut s = [[0.0f32; 3]; 3];
        for i in 0..3 { for j in 0..3 { s[i][j] = self.p[i][j]; } }
        for i in 0..3 { s[i][i] += r_var; }
        let s_inv = match inverser33(&s) { Some(m) => m, None => return };

        // K = P[:, 0:3] · S⁻¹  (P·Hᵀ = premières 3 colonnes de P)
        let mut pht = [[0.0f32; 3]; 13];
        for i in 0..13 { for j in 0..3 { pht[i][j] = self.p[i][j]; } }
        let k = mult_m13x3_m33(&pht, &s_inv);

        for i in 0..13 { self.x[i] += k[i][0]*y[0] + k[i][1]*y[1] + k[i][2]*y[2]; }
        self.normaliser_quat();

        // P = (I − K·H)·P,  K·H a ses 3 premières colonnes = k, reste = 0
        let mut kh = [[0.0f32; 13]; 13];
        for i in 0..13 { for j in 0..3 { kh[i][j] = k[i][j]; } }
        self.p = mult_m13(&soustraire_identite(&kh), &self.p);
    }

    /// Correction avec H_vel = [0 | I₃ | 0 | 0] — structure creuse exploitée.
    fn corriger_vel(&mut self, y: V3, r_var: f32) {
        // S = P[3:6, 3:6] + r·I₃
        let mut s = [[0.0f32; 3]; 3];
        for i in 0..3 { for j in 0..3 { s[i][j] = self.p[3+i][3+j]; } }
        for i in 0..3 { s[i][i] += r_var; }
        let s_inv = match inverser33(&s) { Some(m) => m, None => return };

        // K = P[:, 3:6] · S⁻¹
        let mut pht = [[0.0f32; 3]; 13];
        for i in 0..13 { for j in 0..3 { pht[i][j] = self.p[i][3+j]; } }
        let k = mult_m13x3_m33(&pht, &s_inv);

        for i in 0..13 { self.x[i] += k[i][0]*y[0] + k[i][1]*y[1] + k[i][2]*y[2]; }
        self.normaliser_quat();

        let mut kh = [[0.0f32; 13]; 13];
        for i in 0..13 { for j in 0..3 { kh[i][3+j] = k[i][j]; } }
        self.p = mult_m13(&soustraire_identite(&kh), &self.p);
    }

    /// Correction générique avec H (3×13) arbitraire — accéléromètre.
    fn corriger_generique(&mut self, y: V3, h: M3x13, r_var: f32) {
        let ht  = transposer_3x13(&h);
        let hp  = mult_3x13_m13(&h, &self.p);
        let mut s = mult_3x13_13x3(&hp, &ht);
        for i in 0..3 { s[i][i] += r_var; }
        let s_inv = match inverser33(&s) { Some(m) => m, None => return };

        let pht = mult_m13_13x3(&self.p, &ht);
        let k   = mult_m13x3_m33(&pht, &s_inv);

        for i in 0..13 { self.x[i] += k[i][0]*y[0] + k[i][1]*y[1] + k[i][2]*y[2]; }
        self.normaliser_quat();

        let kh = mult_13x3_3x13(&k, &h);
        self.p = mult_m13(&soustraire_identite(&kh), &self.p);
    }

    fn normaliser_quat(&mut self) {
        let q = self.quat().normalise();
        self.x[6] = q.w; self.x[7] = q.x; self.x[8] = q.y; self.x[9] = q.z;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers — Jacobiens et modèles
// ─────────────────────────────────────────────────────────────────────────────

/// Rotation corps → monde (NED) : v_ned = q ⊗ [0,v] ⊗ q*
fn corps_vers_monde(q: &Quaternion, v: Vector3) -> Vector3 {
    let vq = Quaternion::nouveau(0.0, v.x, v.y, v.z);
    let r  = q.multiplier(&vq).multiplier(&q.conjugue());
    Vector3::nouveau(r.x, r.y, r.z)
}

/// Jacobien F (13×13) du modèle de processus.
fn construire_f(q: &Quaternion, omega: Vector3, a_meas: Vector3, dt: f32) -> M13 {
    let mut f = identite13();
    let dt2h = 0.5 * dt * dt;

    // ∂p/∂v = dt·I₃
    f[0][3] = dt; f[1][4] = dt; f[2][5] = dt;

    // ∂p/∂q = (dt²/2)·J_Ra   et   ∂v/∂q = dt·J_Ra
    let jra = jacobien_ra(q, a_meas);
    for i in 0..3 {
        for j in 0..4 {
            f[i]  [6+j] = dt2h * jra[i][j];
            f[3+i][6+j] = dt   * jra[i][j];
        }
    }

    // ∂q/∂q = F_q
    let fq = omega_vers_fq(omega, dt);
    for i in 0..4 { for j in 0..4 { f[6+i][6+j] = fq[i][j]; } }

    // ∂q/∂bg = -(dt/2)·Q_mat(q)
    let qbg = q_mat_bg(q, dt);
    for i in 0..4 { for j in 0..3 { f[6+i][10+j] = qbg[i][j]; } }

    f
}

/// Jacobien H_accel (3×13) de la mesure accéléromètre.
fn h_accel(q: &Quaternion) -> M3x13 {
    let jh = jacobien_h(q);
    let mut h = [[0.0f32; 13]; 3];
    for i in 0..3 { for j in 0..4 { h[i][6+j] = jh[i][j]; } }
    h
}

/// Jacobien 3×4 de R(q)·a par rapport à [qw, qx, qy, qz].
///
/// R(q) est la matrice de rotation corps → monde (NED).
fn jacobien_ra(q: &Quaternion, a: Vector3) -> M34 {
    let (ax, ay, az)       = (a.x, a.y, a.z);
    let (qw, qx, qy, qz)  = (q.w, q.x, q.y, q.z);
    [
        [2.0*(-qz*ay + qy*az),
         2.0*( qy*ay + qz*az),
         2.0*(-2.0*qy*ax + qx*ay + qw*az),
         2.0*(-2.0*qz*ax - qw*ay + qx*az)],
        [2.0*( qz*ax - qx*az),
         2.0*( qy*ax - 2.0*qx*ay - qw*az),
         2.0*( qx*ax + qz*az),
         2.0*( qw*ax - 2.0*qz*ay + qy*az)],
        [2.0*(-qy*ax + qx*ay),
         2.0*( qz*ax + qw*ay - 2.0*qx*az),
         2.0*(-qw*ax + qz*ay - 2.0*qy*az),
         2.0*( qx*ax + qy*ay)],
    ]
}

/// Jacobien 3×4 de monde_vers_corps(G) par rapport à q.
fn jacobien_h(q: &Quaternion) -> M34 {
    let g2 = 2.0 * G;
    [
        [-g2*q.y,         g2*q.z,        -g2*q.w,        g2*q.x],
        [ g2*q.x,         g2*q.w,         g2*q.z,        g2*q.y],
        [ 0.0,   -4.0*G*q.x,  -4.0*G*q.y,  0.0   ],
    ]
}

/// Jacobien F_q = ∂q_{k+1}/∂q_k (4×4).
fn omega_vers_fq(w: Vector3, dt: f32) -> M44 {
    let h = dt * 0.5;
    [
        [ 1.0,    -h*w.x, -h*w.y, -h*w.z],
        [ h*w.x,  1.0,    h*w.z,  -h*w.y],
        [ h*w.y, -h*w.z,  1.0,    h*w.x ],
        [ h*w.z,  h*w.y, -h*w.x,  1.0   ],
    ]
}

/// ∂q_{k+1}/∂bg = -(dt/2)·[q]_L[:, 1:4]  (4×3).
fn q_mat_bg(q: &Quaternion, dt: f32) -> M43 {
    let h = -dt * 0.5;
    [
        [h*(-q.x), h*(-q.y), h*(-q.z)],
        [h*( q.w), h*(-q.z), h*( q.y)],
        [h*( q.z), h*( q.w), h*(-q.x)],
        [h*(-q.y), h*( q.x), h*( q.w)],
    ]
}

/// Matrice de bruit processus Q (13×13), diagonale par blocs.
fn bruit_processus(dt: f32) -> M13 {
    let mut q = [[0.0f32; 13]; 13];
    for i in 3..6   { q[i][i] = Q_VIT   * dt; }
    for i in 6..10  { q[i][i] = Q_ATT   * dt; }
    for i in 10..13 { q[i][i] = Q_BIAIS * dt; }
    q
}

// ─────────────────────────────────────────────────────────────────────────────
// Conversion géodésique → NED (approximation Terre plate)
// ─────────────────────────────────────────────────────────────────────────────

fn lat_lon_alt_vers_ned(lat: f64, lon: f64, alt: f32, orig: &OrigineNed) -> Vector3 {
    let dlat    = (lat - orig.lat).to_radians();
    let dlon    = (lon - orig.lon).to_radians();
    let cos_lat = orig.lat.to_radians().cos();
    Vector3::nouveau(
        (dlat * R_TERRE) as f32,
        (dlon * R_TERRE * cos_lat) as f32,
        -(alt - orig.alt),
    )
}

// ─────────────────────────────────────────────────────────────────────────────
// Algèbre linéaire
// ─────────────────────────────────────────────────────────────────────────────

fn identite13() -> M13 {
    let mut m = [[0.0f32; 13]; 13];
    for i in 0..13 { m[i][i] = 1.0; }
    m
}

fn mult_m13(a: &M13, b: &M13) -> M13 {
    let mut r = [[0.0f32; 13]; 13];
    for i in 0..13 { for j in 0..13 { for k in 0..13 { r[i][j] += a[i][k]*b[k][j]; } } }
    r
}

fn ajouter_m13(a: &M13, b: &M13) -> M13 {
    let mut r = [[0.0f32; 13]; 13];
    for i in 0..13 { for j in 0..13 { r[i][j] = a[i][j] + b[i][j]; } }
    r
}

fn transposer_m13(m: &M13) -> M13 {
    let mut r = [[0.0f32; 13]; 13];
    for i in 0..13 { for j in 0..13 { r[i][j] = m[j][i]; } }
    r
}

/// I₁₃ − m
fn soustraire_identite(m: &M13) -> M13 {
    let mut r = identite13();
    for i in 0..13 { for j in 0..13 { r[i][j] -= m[i][j]; } }
    r
}

/// H (3×13) · P (13×13) → 3×13
fn mult_3x13_m13(h: &M3x13, p: &M13) -> M3x13 {
    let mut r = [[0.0f32; 13]; 3];
    for i in 0..3 { for j in 0..13 { for k in 0..13 { r[i][j] += h[i][k]*p[k][j]; } } }
    r
}

/// A (3×13) · B (13×3) → 3×3
fn mult_3x13_13x3(a: &M3x13, b: &M13x3) -> M33 {
    let mut r = [[0.0f32; 3]; 3];
    for i in 0..3 { for j in 0..3 { for k in 0..13 { r[i][j] += a[i][k]*b[k][j]; } } }
    r
}

/// P (13×13) · Hᵀ (13×3) → 13×3
fn mult_m13_13x3(p: &M13, ht: &M13x3) -> M13x3 {
    let mut r = [[0.0f32; 3]; 13];
    for i in 0..13 { for j in 0..3 { for k in 0..13 { r[i][j] += p[i][k]*ht[k][j]; } } }
    r
}

/// A (13×3) · B (3×3) → 13×3
fn mult_m13x3_m33(a: &M13x3, b: &M33) -> M13x3 {
    let mut r = [[0.0f32; 3]; 13];
    for i in 0..13 { for j in 0..3 { for k in 0..3 { r[i][j] += a[i][k]*b[k][j]; } } }
    r
}

/// K (13×3) · H (3×13) → 13×13
fn mult_13x3_3x13(k: &M13x3, h: &M3x13) -> M13 {
    let mut r = [[0.0f32; 13]; 13];
    for i in 0..13 { for j in 0..13 { for kk in 0..3 { r[i][j] += k[i][kk]*h[kk][j]; } } }
    r
}

/// Hᵀ : H (3×13) → Hᵀ (13×3)
fn transposer_3x13(h: &M3x13) -> M13x3 {
    let mut r = [[0.0f32; 3]; 13];
    for i in 0..3 { for j in 0..13 { r[j][i] = h[i][j]; } }
    r
}

/// Inversion 3×3 par la règle de Cramer. Retourne `None` si singulière.
fn inverser33(m: &M33) -> Option<M33> {
    let [[a,b,c],[d,e,f],[g,h,k]] = *m;
    let det = a*(e*k-f*h) - b*(d*k-f*g) + c*(d*h-e*g);
    if det.abs() < 1e-9 { return None; }
    let inv = 1.0 / det;
    Some([
        [(e*k-f*h)*inv, (c*h-b*k)*inv, (b*f-c*e)*inv],
        [(f*g-d*k)*inv, (a*k-c*g)*inv, (c*d-a*f)*inv],
        [(d*h-e*g)*inv, (b*g-a*h)*inv, (a*e-b*d)*inv],
    ])
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Horodatage, TypeFixGps};

    fn donnees_gps_test(lat: f64, lon: f64, alt: f32) -> DonneesGps {
        DonneesGps {
            horodatage:       Horodatage::maintenant(),
            latitude:         lat,
            longitude:        lon,
            altitude_msl:     alt,
            vitesse_sol:      0.0,
            cap:              0.0,
            vel_nord:         0.0,
            vel_est:          0.0,
            vel_bas:          0.0,
            precision_h:      2.0,
            precision_v:      3.0,
            nombre_satellites: 8,
            type_fix:         TypeFixGps::Fix3D,
        }
    }

    #[test]
    fn etat_initial_nul() {
        let ekf = EkfNavigation::nouveau();
        let e   = ekf.etat();
        assert!(!e.origine_definie);
        assert!(e.position_ned.norme() < 1e-6);
        assert!(e.vitesse_ned.norme()  < 1e-6);
    }

    #[test]
    fn premiere_prediction_initialise_attitude() {
        let mut ekf = EkfNavigation::nouveau();
        ekf.predire(Vector3::nouveau(0.0, 0.0, G), Vector3::zero(), 0.005);
        let e = ekf.etat();
        assert!(e.attitude.roulis.degres().abs()  < 0.1, "roulis  = {:.3}", e.attitude.roulis.degres());
        assert!(e.attitude.tangage.degres().abs() < 0.1, "tangage = {:.3}", e.attitude.tangage.degres());
    }

    #[test]
    fn repos_attitude_stable() {
        let mut ekf = EkfNavigation::nouveau();
        ekf.predire(Vector3::nouveau(0.0, 0.0, G), Vector3::zero(), 0.005);
        for _ in 0..200 {
            ekf.predire(Vector3::nouveau(0.0, 0.0, G), Vector3::zero(), 0.005);
            ekf.corriger_accel(Vector3::nouveau(0.0, 0.0, G));
        }
        let e = ekf.etat();
        assert!(e.attitude.roulis.degres().abs()  < 0.5, "roulis  = {:.3}", e.attitude.roulis.degres());
        assert!(e.attitude.tangage.degres().abs() < 0.5, "tangage = {:.3}", e.attitude.tangage.degres());
    }

    #[test]
    fn gps_premier_fix_definit_origine() {
        let mut ekf = EkfNavigation::nouveau();
        ekf.predire(Vector3::nouveau(0.0, 0.0, G), Vector3::zero(), 0.005);

        ekf.corriger_gps(&donnees_gps_test(48.8566, 2.3522, 50.0));

        assert!(ekf.etat().origine_definie);
        let p = ekf.etat().position_ned;
        assert!(p.norme() < 1.0, "position ≠ 0 à l'origine : {:?}", p);
    }

    #[test]
    fn gps_deplacement_nord_1km() {
        let mut ekf = EkfNavigation::nouveau();
        ekf.predire(Vector3::nouveau(0.0, 0.0, G), Vector3::zero(), 0.005);

        // Un seul fix pour fixer l'origine (P_pos → ~4 m²)
        ekf.corriger_gps(&donnees_gps_test(48.8566, 2.3522, 50.0));

        // 25 corrections consécutives à +1 km Nord — convergence attendue > 900 m
        for _ in 0..25 {
            ekf.corriger_gps(&donnees_gps_test(48.8656, 2.3522, 50.0));
        }

        let p = ekf.etat().position_ned;
        assert!(p.x > 900.0 && p.x < 1100.0,
            "déplacement Nord attendu ≈ 1000 m, obtenu {:.1} m", p.x);
        assert!(p.y.abs() < 10.0,
            "composante Est attendue ≈ 0 m, obtenu {:.1} m", p.y);
    }

    #[test]
    fn correction_accel_rejetee_vol_accelere() {
        let mut ekf = EkfNavigation::nouveau();
        ekf.predire(Vector3::nouveau(0.0, 0.0, G), Vector3::zero(), 0.005);
        // Initialiser avec roulis 15°
        let angle = 15.0_f32.to_radians();
        ekf.predire(
            Vector3::nouveau(0.0, G*angle.sin(), G*angle.cos()),
            Vector3::zero(), 0.005,
        );
        let roulis_avant = ekf.etat().attitude.roulis.degres();
        // Appliquer 50 corrections avec accélération anormale
        for _ in 0..50 {
            ekf.corriger_accel(Vector3::nouveau(0.0, 0.0, 2.0 * G));
        }
        let roulis_apres = ekf.etat().attitude.roulis.degres();
        assert!((roulis_apres - roulis_avant).abs() < 1.0,
            "correction ne devrait pas s'appliquer : avant={:.2}° après={:.2}°",
            roulis_avant, roulis_apres);
    }
}
