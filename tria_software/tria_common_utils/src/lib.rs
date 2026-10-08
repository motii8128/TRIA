pub mod robomaster;
pub mod pid;

use nalgebra::Isometry2;

extern crate nalgebra;

pub type Vec3 = nalgebra::Vector3<f32>;

pub type Vec2 = nalgebra::Vector2<f32>;

pub fn transform_pointcloud2d(mut point_cloud: Vec<Vec2>, transform: Position2D) -> Vec<Vec2> {
    for p in point_cloud.iter_mut() {
        let transformed = transform
            .iso
            .transform_point(&nalgebra::OPoint { coords: *p });
        p.x = transformed.coords.x;
        p.y = transformed.coords.y;
    }

    point_cloud
}

#[derive(Debug, Clone, Copy)]
pub struct Position2D {
    pub iso: nalgebra::Isometry2<f32>,
}

impl Position2D {
    pub fn new(x: f32, y: f32, yaw: f32) -> Position2D {
        return Position2D {
            iso: Isometry2::new(Vec2::new(x, y), yaw),
        };
    }

    pub fn norm(&self) -> f32 {
        return self.iso.translation.vector.norm();
    }

    pub fn get_position(&self) -> Vec2 {
        Vec2::new(self.iso.translation.x, self.iso.translation.y)
    }

    pub fn get_theta(&self) -> f32 {
        self.iso.rotation.angle()
    }
}
