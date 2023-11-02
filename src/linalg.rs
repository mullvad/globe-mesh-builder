use std::hash::Hash;
use std::ops::Mul;
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

/// Represent a vertex as three integers where the coordinates are rounded off.
/// Used for equality/hash comparison to make vertices close to each other
/// say that they are the same.
fn round_vertex(v: Vertex) -> [i128; 3] {
    const VERTEX_ROUNDING: f32 = 1000.0;
    v.0.into_array().map(|f| (f * VERTEX_ROUNDING) as i128)
}

impl PartialEq for Vertex {
    fn eq(&self, other: &Self) -> bool {
        let self_rounded = round_vertex(*self);
        let other_rounded = round_vertex(*other);
        self_rounded == other_rounded
    }
}

impl Eq for Vertex {}

impl Hash for Vertex {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        round_vertex(*self).hash(state);
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
