use embedded_graphics::{
    prelude::{Angle, PixelColor, Point},
    primitives::{Line, PrimitiveStyle, Styled, StyledDrawable},
    Drawable,
};
use micromath::F32Ext as _;
use nalgebra::{Vector2, Vector3};

// pub const CUBE_VERTICES: [Vector3<f32>; 8] = {
//     let mut vertices = [Vector3::new(0.0, 0.0, 0.0); 8];
//     let mut i = 0;
//     loop {
//         if i == 8 {
//             break;
//         }

//         vertices[i] = Vector3::new(
//             if i % 4 == 0 { -1.0 } else { 1.0 },
//             if i % 3 == 0 { -1.0 } else { 1.0 },
//             if i % 2 == 0 { -1.0 } else { 1.0 },
//         );

//         i += 1;
//     }

//     vertices
// };

pub const CUBE_VERTICES: [Vector3<f32>; 8] = [
    Vector3::new(-1.0, -1.0, -1.0),
    Vector3::new(-1.0, -1.0, 1.0),
    Vector3::new(-1.0, 1.0, -1.0),
    Vector3::new(-1.0, 1.0, 1.0),
    Vector3::new(1.0, -1.0, -1.0),
    Vector3::new(1.0, -1.0, 1.0),
    Vector3::new(1.0, 1.0, -1.0),
    Vector3::new(1.0, 1.0, 1.0),
];

pub const CUBE_EDGES: [Vector2<usize>; 12] = [
    Vector2::new(0, 1),
    Vector2::new(1, 3),
    Vector2::new(3, 2),
    Vector2::new(2, 0),
    Vector2::new(4, 5),
    Vector2::new(5, 7),
    Vector2::new(7, 6),
    Vector2::new(6, 4),
    Vector2::new(0, 4),
    Vector2::new(1, 5),
    Vector2::new(2, 6),
    Vector2::new(3, 7),
];

// TODO: Fill, 2d FOV projection

pub struct Cube {
    pos: Point,
    size: u32,
    vertices: [Vector3<f32>; 8],
    // fov: f32,
}

impl Cube {
    pub fn new(pos: Point, size: u32) -> Self {
        Self {
            pos,
            size,
            vertices: CUBE_VERTICES.map(|vertex| vertex * size as f32),
        }
    }

    pub fn rotate(&mut self, angle_x: Angle, angle_y: Angle) {
        let (sin_x, cos_x) = angle_x.to_radians().sin_cos();
        let (sin_y, cos_y) = angle_y.to_radians().sin_cos();

        self.vertices.iter_mut().for_each(|vertex| {
            let x = vertex.x * cos_x - vertex.z * sin_x;
            let z = vertex.z * cos_x + vertex.x * sin_x;

            let y = vertex.y * cos_y - z * sin_y;
            let z = z * cos_y + vertex.y * sin_y;

            *vertex = Vector3::new(x, y, z)
        });
    }

    pub fn rotated(mut self, angle_x: Angle, angle_y: Angle) -> Self {
        self.rotate(angle_x, angle_y);
        self
    }

    pub fn rotate_3d(&mut self, angle_x: Angle, angle_y: Angle, angle_z: Angle) {
        let (sin_x, cos_x) = angle_x.to_radians().sin_cos();
        let (sin_y, cos_y) = angle_y.to_radians().sin_cos();
        let (sin_z, cos_z) = angle_z.to_radians().sin_cos();

        self.vertices.iter_mut().for_each(|vertex| {
            // Rotate X
            let [_, y, z] = (*vertex).into();
            vertex.y = y * cos_x - z * sin_x;
            vertex.z = y * sin_x + z * cos_x;
            // Rotate Y
            let [x, _, z] = (*vertex).into();
            vertex.x = x * cos_y + z * sin_y;
            vertex.z = -x * sin_y + z * cos_y;
            // Rotate Z
            let [x, y, _] = (*vertex).into();
            vertex.x = x * cos_z - y * sin_z;
            vertex.y = x * sin_z + y * cos_z;
        });
    }

    pub fn rotated_3d(mut self, angle_x: Angle, angle_y: Angle, angle_z: Angle) -> Self {
        self.rotate_3d(angle_x, angle_y, angle_z);
        self
    }

    pub fn vertex(&self, index: usize) -> Option<Vector3<f32>> {
        self.vertices.get(index).copied()
    }

    pub fn edges(&self) -> impl Iterator<Item = Line> + '_ {
        (0..CUBE_EDGES.len()).map(|edge| self.edge_n(edge).unwrap())
    }

    pub fn edge_n(&self, edge: usize) -> Option<Line> {
        CUBE_EDGES.get(edge).map(|edge| {
            let start = self.vertices[edge.x];
            let end = self.vertices[edge.y];
            Line::new(
                Point::new(start.x as i32, start.y as i32) + self.pos,
                Point::new(end.x as i32, end.y as i32) + self.pos,
            )
        })
    }
}

impl<C: PixelColor> StyledDrawable<PrimitiveStyle<C>> for Cube {
    type Color = C;
    type Output = ();

    fn draw_styled<D>(
        &self,
        style: &PrimitiveStyle<C>,
        target: &mut D,
    ) -> Result<Self::Output, D::Error>
    where
        D: embedded_graphics::prelude::DrawTarget<Color = Self::Color>,
    {
        self.edges()
            .try_for_each(|edge| edge.draw_styled(style, target))
    }
}
