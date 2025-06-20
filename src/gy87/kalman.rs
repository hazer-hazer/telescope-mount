// use micromath::F32Ext;
// use nalgebra::{
//     Matrix3, Matrix4, OMatrix, OVector, Quaternion, UnitQuaternion, Vector3, Vector4, U3, U4,
// };

// // Custom error handling
// #[derive(Debug, PartialEq)]
// pub enum KalmanError {
//     InvalidMeasurement,
//     NonPositiveDefinite,
//     NormalizationFailed,
// }

// // State: Orientation (quaternion) + Gyro bias
// #[repr(C)]
// #[derive(Debug, Clone, Copy)]
// pub struct ImuState {
//     pub q: Vector4<f32>,         // [w, x, y, z] quaternion
//     pub gyro_bias: Vector3<f32>, // Gyroscope bias
// }

// // IMU Kalman Filter (Fixed-size for embedded)
// pub struct KalmanFilter {
//     // State transition (F)
//     state_transition: Matrix4<f32>,
//     // Process noise (Q)
//     process_noise: Matrix4<f32>,
//     // Measurement matrices (H_accel, H_mag)
//     h_accel: Matrix3<f32>,
//     h_mag: Matrix3<f32>,
//     // Sensor noise (R_accel, R_mag)
//     r_accel: Matrix3<f32>,
//     r_mag: Matrix3<f32>,
// }

// impl KalmanFilter {
//     /// Creates a new Kalman filter with IMU-specific parameters
//     pub fn new(gyro_noise: f32, gyro_bias_noise: f32, accel_noise: f32, mag_noise: f32) -> Self {
//         // State transition (simplified quaternion integration)
//         let state_transition = Matrix4::identity();

//         // Process noise (gyro noise + bias drift)
//         let process_noise = Matrix4::from_diagonal(&Vector4::new(
//             gyro_noise,
//             gyro_noise,
//             gyro_noise,
//             gyro_bias_noise,
//         ));

//         // Measurement models (gravity and magnetic field)
//         let h_accel = Matrix3::identity();
//         let h_mag = Matrix3::identity();

//         // Sensor noise covariances
//         let r_accel = Matrix3::from_diagonal(&Vector3::repeat(accel_noise));
//         let r_mag = Matrix3::from_diagonal(&Vector3::repeat(mag_noise));

//         Self {
//             state_transition,
//             process_noise,
//             h_accel,
//             h_mag,
//             r_accel,
//             r_mag,
//         }
//     }

//     /// Initialize state from first accelerometer/magnetometer reading
//     pub fn initialize(
//         &self,
//         accel: &Vector3<f32>,
//         mag: &Vector3<f32>,
//     ) -> Result<ImuState, KalmanError> {
//         let q = Self::compute_initial_quaternion(accel, mag)?;
//         Ok(ImuState {
//             q,
//             gyro_bias: Vector3::zeros(),
//         })
//     }

//     /// Madgwick-style initial quaternion from accel/mag
//     fn compute_initial_quaternion(
//         accel: &Vector3<f32>,
//         mag: &Vector3<f32>,
//     ) -> Result<Vector4<f32>, KalmanError> {
//         // Normalize and orthogonalize vectors
//         let a_norm = accel.normalize();
//         let m_norm = mag.normalize();

//         // Cross product for orthogonal reference frame
//         let east = a_norm.cross(&m_norm).normalize();
//         let north = a_norm.cross(&east).normalize();

//         // Quaternion from rotation matrix
//         let rot_mat = Matrix3::from_columns(&[east, north, a_norm]);
//         Ok(UnitQuaternion::from_matrix(&rot_mat).coords)
//     }

//     /// Predict state using gyroscope data (dt in seconds)
//     pub fn predict(&self, state: &ImuState, gyro: &Vector3<f32>, dt: f32) -> ImuState {
//         // Gyro correction (remove bias)
//         let corrected_gyro = gyro - state.gyro_bias;

//         // Quaternion derivative (dq/dt = 0.5 * q ⊗ [0, ω])
//         let omega = Vector4::new(0.0, corrected_gyro.x, corrected_gyro.y, corrected_gyro.z);
//         let q_dot = 0.5 * state.q * omega;

//         // Update quaternion (first-order integration)
//         let new_q = (state.q + q_dot * dt).normalize();

//         // New state (assume bias constant)
//         ImuState {
//             q: new_q,
//             gyro_bias: state.gyro_bias,
//         }
//     }

//     /// Update with accelerometer (gravity vector)
//     pub fn update_accel(
//         &self,
//         state: &ImuState,
//         accel: &Vector3<f32>,
//     ) -> Result<ImuState, KalmanError> {
//         // Predicted gravity vector from quaternion
//         let predicted = self.h_accel * state.q * Vector3::z() * state.q.conjugate();

//         // Innovation
//         let y = accel - predicted;
//         let s = self.h_accel * state.covariance * self.h_accel.transpose() + self.r_accel;

//         // Kalman gain
//         let k = state.covariance
//             * self.h_accel.transpose()
//             * s.try_inverse().ok_or(KalmanError::NonPositiveDefinite)?;

//         // State update
//         let delta = k * y;
//         Ok(ImuState {
//             q: (state.q + delta.fixed_rows::<4>(0)).normalize(),
//             gyro_bias: state.gyro_bias + delta.fixed_rows::<3>(4),
//         })
//     }

//     /// Update with magnetometer (north vector)
//     pub fn update_mag(
//         &self,
//         state: &ImuState,
//         mag: &Vector3<f32>,
//     ) -> Result<ImuState, KalmanError> {
//         // Similar to accel but with magnetic field model
//         // (Implementation omitted for brevity)
//         unimplemented!()
//     }
// }
