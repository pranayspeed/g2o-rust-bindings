extern crate g2orust;
use nalgebra::{Isometry3};
use cxx::{let_cxx_string};


pub struct Pose {
    pose: Isometry3<f64>
}

impl Pose {
    pub fn default() -> Pose {
        Pose {
            pose : Isometry3::identity() 
        }
    }

    pub fn t_to_array(&self) -> [f64; 3] {
        [self.pose.translation.x, self.pose.translation.y, self.pose.translation.z]
    }

    pub fn r_quaternion_to_array(&self) -> [f64; 4] {
        [self.pose.rotation.w, self.pose.rotation.i, self.pose.rotation.j, self.pose.rotation.k]
    }
}

fn main() {
    let optimizer_ptr = g2orust::ffi::new_sparse_optimizer();

    let pose = Pose::default();
    let_cxx_string!(vertex_name = "VertexSE3Expmap");

    g2orust::ffi::create_frame_vertex(
        1, &vertex_name, 
        pose.t_to_array(), pose.r_quaternion_to_array(), 
        optimizer_ptr
    );

}
