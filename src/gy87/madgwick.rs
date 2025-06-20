use embassy_time::Instant;
use micromath::F32Ext as _;
use nalgebra::{SVector, Vector3, Vector4};

#[derive(Clone, Copy)]
pub struct Rotation {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

pub struct MadgwickFilter {
    q: Vector4<f32>, // Quaternion [w, x, y, z]
    beta: f32,       // Algorithm gain
    last_update: Instant,
}

impl MadgwickFilter {
    pub fn new(beta: f32) -> Self {
        Self {
            q: Vector4::new(1.0, 0.0, 0.0, 0.0), // Initial quaternion (no rotation)
            beta,
            last_update: Instant::now(),
        }
    }

    pub fn update(
        &mut self,
        accel: Vector3<f32>,
        gyro: Vector3<f32>,
        mag: Vector3<f32>,
    ) -> Option<Vector4<f32>> {
        let delta_t = (Instant::now() - self.last_update).as_micros() as f32 / 1e6;

        let [ax, ay, az] = accel.normalize().into();
        let [mx, my, mz] = mag.normalize().into();
        let [gx, gy, gz] = gyro.into();

        let [q1, q2, q3, q4] = self.q.into();

        // Auxiliary variables to avoid repeated arithmetic
        let two_q1 = 2.0 * q1;
        let two_q2 = 2.0 * q2;
        let two_q3 = 2.0 * q3;
        let two_q4 = 2.0 * q4;
        let two_q1q3 = two_q1 * q3;
        let two_q3q4 = two_q3 * q4;

        let q1q1 = q1 * q1;
        let q1q2 = q1 * q2;
        let q1q3 = q1 * q3;
        let q1q4 = q1 * q4;
        let q2q2 = q2 * q2;
        let q2q3 = q2 * q3;
        let q2q4 = q2 * q4;
        let q3q3 = q3 * q3;
        let q3q4 = q3 * q4;
        let q4q4 = q4 * q4;

        // Reference direction of Earth's magnetic field
        let two_q1mx = two_q1 * mx;
        let two_q1my = two_q1 * my;
        let two_q1mz = two_q1 * mz;
        let two_q2mx = two_q2 * mx;

        let hx = mx * q1q1 - two_q1my * q4
            + two_q1mz * q3
            + mx * q2q2
            + two_q2 * my * q3
            + two_q2 * mz * q4
            - mx * q3q3
            - mx * q4q4;

        let hy = two_q1mx * q4 + my * q1q1 - two_q1mz * q2 + two_q2mx * q3 - my * q2q2
            + my * q3q3
            + two_q3 * mz * q4
            - my * q4q4;

        let two_bx = (hx * hx + hy * hy).sqrt();
        let two_bz = -two_q1mx * q3 + two_q1my * q2 + mz * q1q1 + two_q2mx * q4 - mz * q2q2
            + two_q3 * my * q4
            - mz * q3q3
            + mz * q4q4;

        let four_bx = 2.0 * two_bx;
        let four_bz = 2.0 * two_bz;

        // Gradient descent algorithm corrective step
        let s1 = -two_q3 * (2.0 * q2q4 - two_q1q3 - ax) + two_q2 * (2.0 * q1q2 + two_q3q4 - ay)
            - two_bz * q3 * (two_bx * (0.5 - q3q3 - q4q4) + two_bz * (q2q4 - q1q3) - mx)
            + (-two_bx * q4 + two_bz * q2) * (two_bx * (q2q3 - q1q4) + two_bz * (q1q2 + q3q4) - my)
            + two_bx * q3 * (two_bx * (q1q3 + q2q4) + two_bz * (0.5 - q2q2 - q3q3) - mz);

        let s2 = two_q4 * (2.0 * q2q4 - two_q1q3 - ax) + two_q1 * (2.0 * q1q2 + two_q3q4 - ay)
            - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az)
            + two_bz * q4 * (two_bx * (0.5 - q3q3 - q4q4) + two_bz * (q2q4 - q1q3) - mx)
            + (two_bx * q3 + two_bz * q1) * (two_bx * (q2q3 - q1q4) + two_bz * (q1q2 + q3q4) - my)
            + (two_bx * q4 - four_bz * q2)
                * (two_bx * (q1q3 + q2q4) + two_bz * (0.5 - q2q2 - q3q3) - mz);

        let s3 = -two_q1 * (2.0 * q2q4 - two_q1q3 - ax) + two_q4 * (2.0 * q1q2 + two_q3q4 - ay)
            - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az)
            + (-four_bx * q3 - two_bz * q1)
                * (two_bx * (0.5 - q3q3 - q4q4) + two_bz * (q2q4 - q1q3) - mx)
            + (two_bx * q2 + two_bz * q4) * (two_bx * (q2q3 - q1q4) + two_bz * (q1q2 + q3q4) - my)
            + (two_bx * q1 - four_bz * q3)
                * (two_bx * (q1q3 + q2q4) + two_bz * (0.5 - q2q2 - q3q3) - mz);

        let s4 = two_q2 * (2.0 * q2q4 - two_q1q3 - ax)
            + two_q3 * (2.0 * q1q2 + two_q3q4 - ay)
            + (-four_bx * q4 + two_bz * q2)
                * (two_bx * (0.5 - q3q3 - q4q4) + two_bz * (q2q4 - q1q3) - mx)
            + (-two_bx * q1 + two_bz * q3) * (two_bx * (q2q3 - q1q4) + two_bz * (q1q2 + q3q4) - my)
            + two_bx * q2 * (two_bx * (q1q3 + q2q4) + two_bz * (0.5 - q2q2 - q3q3) - mz);

        let [s1, s2, s3, s4] = Vector4::new(s1, s2, s3, s4).normalize().into();

        // Compute rate of change of quaternion
        let q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1;
        let q_dot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2;
        let q_dot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3;
        let q_dot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4;

        // Integrate to yield quaternion
        self.q[0] = q1 + q_dot1 * delta_t;
        self.q[1] = q2 + q_dot2 * delta_t;
        self.q[2] = q3 + q_dot3 * delta_t;
        self.q[3] = q4 + q_dot4 * delta_t;

        self.last_update = Instant::now();

        // Normalize quaternion
        Some(self.q.normalize())
    }

    pub fn quaternion(&self) -> &Vector4<f32> {
        &self.q
    }

    pub fn rotation(&self) -> Rotation {
        let [q0, q1, q2, q3] = self.q.into();
        Rotation {
            yaw: (2.0 * (q1 * q2 + q0 * q3)).atan2(q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3),
            pitch: -(2.0 * (q1 * q3 - q0 * q2).asin()),
            roll: (2.0 * (q0 * q1 + q2 * q3)).atan2(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3),
        }
    }
}
