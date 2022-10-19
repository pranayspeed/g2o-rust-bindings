
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../core/robust_kernel_impl.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../solvers/linear_solver_dense.h"
#include "../types/se3quat.h"
#include "../types/types_six_dof_expmap.h"
#include "../../../target/cxxbridge/g2orust/src/lib.rs.h"

namespace g2o {
    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer() {
        return std::unique_ptr<BridgeSparseOptimizer>(new BridgeSparseOptimizer());
    }

    BridgeSparseOptimizer::BridgeSparseOptimizer() {
        // ORB_SLAM3 Optimizer::PoseOptimization lines 817-825
        optimizer = std::make_unique<SparseOptimizer>();
        linearSolver = new LinearSolverDense<BlockSolver_6_3::PoseMatrixType>();
        solver_ptr = new BlockSolver_6_3(linearSolver);
        solver = new OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer->setAlgorithm(solver);

        deltaMono = sqrt(5.991);
        deltaStereo = sqrt(7.815);
    }

    // Sofiya note: This might already be deleted when g2o deletes the sparseoptimizer?
    // Sofiya TODO: should check memory leaks though
    // BridgeSparseOptimizer::~BridgeSparseOptimizer() {
    //     delete linearSolver;
    //     delete solver_ptr;
    //     delete solver;
    // }
 
    std::shared_ptr<VertexSE3Expmap> BridgeSparseOptimizer::create_frame_vertex (
        int vertex_id, 
        Pose pose
    ) const {
        // ORB_SLAM3 Optimizer::PoseOptimization lines 830-835
        std::shared_ptr<VertexSE3Expmap> vSE3 = std::make_shared<VertexSE3Expmap>();
        this->set_vertex_estimate(vSE3, pose);

        vSE3->setId(vertex_id);
        vSE3->setFixed(false);
        optimizer->addVertex(vSE3.get());
        return vSE3;
    }

    void BridgeSparseOptimizer::set_vertex_estimate(
        std::shared_ptr<VertexSE3Expmap> vertex,
        Pose pose
    ) const {
        Eigen::Vector3d trans_vec(pose.translation.data());
        Eigen::Quaterniond rot_quat(pose.rotation.data());
        // Rotation is already a quaternion in Darvis!!
        // Don't need to do the conversion in ORBSLAM3
        vertex->setEstimate(SE3Quat(rot_quat, trans_vec));
    }

    std::shared_ptr<BridgeEdgeSE3ProjectXYZOnlyPose> BridgeSparseOptimizer::create_edge_monocular(
        int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y,
        float invSigma2
    ) const {
        // ORB_SLAM3 Optimizer::PoseOptimization lines 869-893
        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;

        std::shared_ptr<EdgeSE3ProjectXYZOnlyPose> edge = std::make_shared<EdgeSE3ProjectXYZOnlyPose>();

        edge->setVertex(
            0, 
            dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(0))
        );
        edge->setMeasurement(obs);
        edge->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        RobustKernelHuber* rk = new RobustKernelHuber;
        edge->setRobustKernel(rk);
        rk->setDelta(deltaMono);

        std::shared_ptr<BridgeEdgeSE3ProjectXYZOnlyPose> bridge_edge(new BridgeEdgeSE3ProjectXYZOnlyPose);
        bridge_edge->edge = edge;
        return bridge_edge;
    }

    void BridgeSparseOptimizer::add_edge_monocular(
        int mp_world_index, std::shared_ptr<BridgeEdgeSE3ProjectXYZOnlyPose> bridge_edge,
        array<double, 3> mp_world_position
    ) const {
        Eigen::Vector3d worldpos_vec(mp_world_position.data());
        bridge_edge->edge->Xw = worldpos_vec;

        optimizer->addEdge(bridge_edge->edge.get());

        // Sofiya...not copied here is lines 931-996
        // "SLAM with respect to a rigid body"
        // Is alternative to add_edge_monocular and add_edge_stereo
        // but not sure why it would be used
    }

    void BridgeSparseOptimizer::_add_edge_stereo() const {
        // TODO (Stereo)
        // ORB_SLAM3 Optimizer::PoseOptimization lines 897-928

        // Eigen::Matrix<double,3,1> obs;
        // const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
        // const float &kp_ur = pFrame->mvuRight[i];
        // obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

        // g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

        // e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        // e->setMeasurement(obs);
        // const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
        // Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
        // e->setInformation(Info);

        // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        // e->setRobustKernel(rk);
        // rk->setDelta(deltaStereo);

        // e->fx = pFrame->fx;
        // e->fy = pFrame->fy;
        // e->cx = pFrame->cx;
        // e->cy = pFrame->cy;
        // e->bf = pFrame->mbf;
        // e->Xw = pMP->GetWorldPos().cast<double>();

        // optimizer.addEdge(e);

        // vpEdgesStereo.push_back(e);
        // vnIndexEdgeStereo.push_back(i);
    }

    void BridgeSparseOptimizer::optimize(int iterations) const {
        optimizer->initializeOptimization(0);
        optimizer->optimize(iterations);
    }

    Pose BridgeSparseOptimizer::recover_optimized_pose() const {
        g2o::VertexSE3Expmap* vSE3_recov = static_cast<VertexSE3Expmap*>(optimizer->vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        Vector3d translation = SE3quat_recov.translation();
        Quaterniond rotation = SE3quat_recov.rotation();

        // Sofiya TODO: make sure that quaternion order of (w,x,y,z)
        // is the order that we use for quaternions in darvis
        // Also, feel like there should be a cleaner way to do this?
        Pose pose;
        pose.translation = {
            (double) translation.x(),
            (double) translation.y(),
            (double) translation.z()
        };
        pose.rotation = {
            (double) rotation.w(), 
            (double) rotation.x(),
            (double) rotation.y(),
            (double) rotation.z()
        };
        return pose;
    }

    int BridgeSparseOptimizer::num_edges() const {
        return optimizer->edges().size();
    }

    void BridgeEdgeSE3ProjectXYZOnlyPose::set_level(int level) const {edge->setLevel(1);}
    void BridgeEdgeSE3ProjectXYZOnlyPose::compute_error() const {edge->computeError();}
    double BridgeEdgeSE3ProjectXYZOnlyPose::chi2() const {return edge->chi2();}
    void BridgeEdgeSE3ProjectXYZOnlyPose::set_robust_kernel(bool reset) const {
        // Sofiya TODO
        // setRobustKernel takes a RobustKernelHuber pointer
        // ORBSLAM3 usually does this but occasionally passes in a 0 instead
        // Here is an alternative implementation that takes a boolean:
        // http://docs.ros.org/en/fuerte/api/re_vision/html/optimizable__graph_8h_source.html
        // although this implementation isn't in the ORBSLAM3 modified g2o...
        // so I have no idea how they are passing in a 0 and compiling it correctly.
        // I *think* that passing in a 0 is equivalent to removing the robust kernel pointer,
        // but need to figure this out...
        if (reset) {
            edge->setRobustKernel(NULL);
        } else {
            RobustKernelHuber* rk = new RobustKernelHuber;
            edge->setRobustKernel(rk);
        }
    }

} // end namespace
