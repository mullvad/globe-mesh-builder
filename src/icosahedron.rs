use crate::geo::Coordinate;
use crate::{Triangle, Vertex};
use std::f32::consts::{FRAC_PI_2, TAU};

pub fn icosahedron_vertices(subdivide_times: u8, scale: f32) -> Vec<Vertex> {
    let mut triangles = icosahedron_faces();
    for _ in 0..subdivide_times {
        let old_triangles = core::mem::replace(&mut triangles, Vec::new());
        for triangle in old_triangles {
            triangles.extend(&subdivide_sphere_face(triangle));
        }
    }

    let mut vertices = Vec::with_capacity(triangles.len() * 3);
    for triangle in triangles {
        vertices.extend(triangle.to_vertices().map(|v| v * scale));
    }
    vertices
}

fn icosahedron_faces() -> Vec<Triangle> {
    let latlong2xyz = |lat: f32, long: f32| crate::latlong2xyz(Coordinate { lat, long });

    let mut triangles = Vec::with_capacity(20);

    let upper_ring_lat = 0.5f32.atan();
    let lower_ring_lat = -upper_ring_lat;
    for i in 0..5 {
        let upper_long1 = i as f32 / 5.0 * TAU;
        let upper_long2 = (i + 1) as f32 / 5.0 * TAU;
        let lower_long1 = (i as f32 + 0.5) / 5.0 * TAU;
        let lower_long2 = (i as f32 + 1.5) / 5.0 * TAU;
        let top_triangle = Triangle::from([
            latlong2xyz(FRAC_PI_2, 0.),
            latlong2xyz(upper_ring_lat, upper_long1),
            latlong2xyz(upper_ring_lat, upper_long2),
        ]);
        let middle_triangle1 = Triangle::from([
            latlong2xyz(upper_ring_lat, upper_long1),
            latlong2xyz(lower_ring_lat, lower_long1),
            latlong2xyz(upper_ring_lat, upper_long2),
        ]);
        let middle_triangle2 = Triangle::from([
            latlong2xyz(upper_ring_lat, upper_long2),
            latlong2xyz(lower_ring_lat, lower_long1),
            latlong2xyz(lower_ring_lat, lower_long2),
        ]);
        let bottom_triangle = Triangle::from([
            latlong2xyz(lower_ring_lat, lower_long1),
            latlong2xyz(-FRAC_PI_2, 0.),
            latlong2xyz(lower_ring_lat, lower_long2),
        ]);
        triangles.push(top_triangle);
        triangles.push(middle_triangle1);
        triangles.push(middle_triangle2);
        triangles.push(bottom_triangle);
    }
    triangles
}

fn subdivide_sphere_face(triangle: Triangle) -> [Triangle; 4] {
    let [v0, v1, v2] = triangle.to_vertices().map(|v| v.to_vector());
    let v3 = Vertex::from((0.5 * (v0 + v1)).normalize());
    let v4 = Vertex::from((0.5 * (v1 + v2)).normalize());
    let v5 = Vertex::from((0.5 * (v2 + v0)).normalize());
    [
        Triangle::from([v0.into(), v3, v5]),
        Triangle::from([v3, v1.into(), v4]),
        Triangle::from([v4, v2.into(), v5]),
        Triangle::from([v3, v4, v5]),
    ]
}
