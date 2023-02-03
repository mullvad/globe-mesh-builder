use std::hash::Hash;
use std::ops::Mul;
use total_float_wrap::TotalF32;
pub use vecmat::Vector;

/// A struct representing a vertex in 3D space.
#[derive(Copy, Clone, Debug)]
pub struct Vertex(Vector<f32, 3>);

impl Vertex {
    pub fn to_vector(self) -> Vector<f32, 3> {
        self.0
    }
}

impl From<[f32; 3]> for Vertex {
    fn from(array: [f32; 3]) -> Self {
        Self::from(Vector::from(array))
    }
}

impl From<Vector<f32, 3>> for Vertex {
    fn from(vector: Vector<f32, 3>) -> Self {
        Vertex(vector)
    }
}

impl Mul<f32> for Vertex {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Vertex(self.0 * rhs)
    }
}

impl PartialEq for Vertex {
    fn eq(&self, other: &Self) -> bool {
        (self.0 - other.0).length() <= f32::EPSILON
    }
}

impl Eq for Vertex {}

impl Hash for Vertex {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        let [v0, v1, v2] = self.0.into_array();
        TotalF32(v0).hash(state);
        TotalF32(v1).hash(state);
        TotalF32(v2).hash(state);
    }
}

/// An triangle in 3 dimensions.
#[derive(Copy, Clone, Debug)]
pub struct Triangle([Vertex; 3]);

impl Triangle {
    pub fn to_vertices(self) -> [Vertex; 3] {
        self.0
    }
}

impl From<[Vertex; 3]> for Triangle {
    fn from(vertices: [Vertex; 3]) -> Self {
        Triangle(vertices)
    }
}
