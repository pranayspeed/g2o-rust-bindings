
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../solvers/linear_solver_dense.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../types/types_six_dof_expmap.h"

namespace g2o {
    struct Pose;

    class BridgeEdgeSE3ProjectXYZOnlyPose{
        public:
            std::shared_ptr<EdgeSE3ProjectXYZOnlyPose> edge;

            void set_level(int level) const;
            void compute_error() const;
            double chi2() const;
            void set_robust_kernel(bool reset) const;
    };

    class BridgeSparseOptimizer {
    public:
        BridgeSparseOptimizer();
        // ~BridgeSparseOptimizer();

        // vertices
        std::shared_ptr<VertexSE3Expmap> create_frame_vertex (
            int vertex_id, Pose pose
        ) const;
        void set_vertex_estimate(
            std::shared_ptr<VertexSE3Expmap> vertex, 
            Pose pose
        ) const;

        // edges
        std::shared_ptr<BridgeEdgeSE3ProjectXYZOnlyPose> create_edge_monocular(
            int keypoint_octave, 
            float keypoint_pt_x, float keypoint_pt_y,
            float invSigma2
        ) const;
        void add_edge_monocular(
            int mp_world_index, 
            std::shared_ptr<BridgeEdgeSE3ProjectXYZOnlyPose> edge,
            array<double, 3> mp_world_position
        ) const;
        void _add_edge_stereo() const;
        int num_edges() const;

        // optimization
        void optimize(int iterations) const;
        Pose recover_optimized_pose() const;

    private:
        std::unique_ptr<SparseOptimizer> optimizer;
        BlockSolver_6_3::LinearSolverType * linearSolver;
        BlockSolver_6_3* solver_ptr;
        OptimizationAlgorithmLevenberg* solver;
        float deltaMono;
        float deltaStereo;
        vector<size_t> vnIndexEdgeMono;
    };

    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer();
} // end namespace
