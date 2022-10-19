#[cxx::bridge(namespace = "g2o")]
pub mod ffi {
    // Shared structs with fields visible to both languages.

    // Sofiya TODO:
    // needed for "SLAM with respect to rigid body" which I am not doing yet
    // struct BridgeEdgeSE3ProjectXYZOnlyPose {
    //     edge: SharedPtr<EdgeSE3ProjectXYZOnlyPose>
    // } 

    struct Pose {
        translation: [f64; 3], // in C++: array<double, 3>,
        rotation: [f64; 4] // in C++: array<double, 4> 
    }

    unsafe extern "C++" {
        // Note: can't use relative path because cargo hates it :(

        include!("rust_helper.h");
        // Opaque types which both languages can pass around
        // but only C++ can see the fields.
        type BridgeSparseOptimizer;
        type VertexSE3Expmap;
        type BridgeEdgeSE3ProjectXYZOnlyPose;

        fn new_sparse_optimizer() -> UniquePtr<BridgeSparseOptimizer>;

        // creating/adding vertices to graph
        fn create_frame_vertex(
            self: &BridgeSparseOptimizer,
            vertex_id: i32,
            pose: Pose,
        ) -> SharedPtr<VertexSE3Expmap>;
        fn set_vertex_estimate(
            self: &BridgeSparseOptimizer,
            vertex: SharedPtr<VertexSE3Expmap>,
            pose: Pose,
        );

        // creating/adding edges to graph
        fn create_edge_monocular(
            self: &BridgeSparseOptimizer,
            keypoint_octave: i32,
            keypoint_pt_x: f32,
            keypoint_pt_y: f32,
            invSigma2: f32,
        ) -> SharedPtr<BridgeEdgeSE3ProjectXYZOnlyPose>;
        fn add_edge_monocular(
            self: &BridgeSparseOptimizer,
            mp_world_index: i32,
            edge: SharedPtr<BridgeEdgeSE3ProjectXYZOnlyPose>,
            mp_world_position: [f64; 3]
        );
        fn _add_edge_stereo(self: &BridgeSparseOptimizer);
        fn num_edges(self: &BridgeSparseOptimizer) -> i32;

        // optimization
        fn optimize(
            self: &BridgeSparseOptimizer,
            iterations: i32,
        );
        fn recover_optimized_pose(
            self: &BridgeSparseOptimizer,
        ) -> Pose;

        // optimization within edge
        fn set_level(
            self: &BridgeEdgeSE3ProjectXYZOnlyPose,
            level: i32,
        );
        fn compute_error(self: &BridgeEdgeSE3ProjectXYZOnlyPose);
        fn chi2(self: &BridgeEdgeSE3ProjectXYZOnlyPose) -> f64;
        fn set_robust_kernel(self: &BridgeEdgeSE3ProjectXYZOnlyPose, reset: bool);
    }
}