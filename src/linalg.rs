use num_traits::float::Float;
use std::hash::Hash;
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

impl From<Vector<f32, 3>> for Vertex {
    fn from(vector: Vector<f32, 3>) -> Self {
        Vertex(vector)
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

/// An triangle in N dimensions. Consists of three vectors with float
/// precision T
#[derive(Copy, Clone, Debug)]
pub struct Triangle<T: Float, const N: usize>([Vector<T, N>; 3]);

impl<T: Float, const N: usize> Triangle<T, N> {
    pub fn to_vertices(self) -> [Vector<T, N>; 3] {
        self.0
    }
}

impl<T: Float, const N: usize> From<[Vector<T, N>; 3]> for Triangle<T, N> {
    fn from(vertices: [Vector<T, N>; 3]) -> Self {
        Triangle(vertices)
    }
}

impl<T: Float, const N: usize> From<Triangle<T, N>> for [Vector<T, N>; 3] {
    fn from(triangle: Triangle<T, N>) -> Self {
        triangle.0
    }
}