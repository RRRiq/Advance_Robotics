#include "kinematics.hpp"

Kinematics::Kinematics(const float &L1, const float &L2, const float &L3)
    : L1_(L1), L2_(L2), L3_(L3)
{
}

Eigen::Vector2f Kinematics::compute_ee_pos(const float &q1, const float &q2, const float& q3)
{
    /**
     * TODO: Compute end effector position using trigonometrics
     */
    float x = L1_ * std::cos(q1)
            + L2_ * std::cos(q1 + q2)
            + L3_ * std::cos(q1 + q2 + q3);

    float y = L1_ * std::sin(q1)
            + L2_ * std::sin(q1 + q2)
            + L3_ * std::sin(q1 + q2 + q3);


    return Eigen::Vector2f(x,y);
}

Eigen::Matrix3f Kinematics::compute_fk_eigen(const float &q1, const float &q2, const float& q3)
{
    // Construct transformation matrices from each joint to next one
    Eigen::Matrix3f T_base_1, T_1_2, T_2_3, T_3_ee;

    /**
     * TODO: Populate the transformation matrices from joint to joint,
     * then chain them to get the transformation from 'base' to 'end effector'.
     */

    T_base_1 << std::cos(q1), -std::sin(q1), L1_ * std::cos(q1),
                std::sin(q1),  std::cos(q1), L1_ * std::sin(q1),
                0,              0,           1;

    T_1_2 << std::cos(q2), -std::sin(q2), L2_ * std::cos(q2),
             std::sin(q2),  std::cos(q2), L2_ * std::sin(q2),
             0,              0,           1;

    T_2_3 << std::cos(q3), -std::sin(q3), L3_ * std::cos(q3),
             std::sin(q3),  std::cos(q3), L3_ * std::sin(q3),
             0,              0,           1;

    Eigen::Matrix3f T_base_ee = T_base_1 * T_1_2 * T_2_3;

    return T_base_ee;
}

void Kinematics::construct_kdl_chain()
{
    /**
     * TODO: Construct KDL Chain object
     */
    chain_.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Vector(L1_, 0, 0))
    ));

    chain_.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Vector(L2_, 0, 0))
    ));

    chain_.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Vector(L3_, 0, 0))
    ));
}


KDL::Frame Kinematics::compute_fk_kdl(const float &q1, const float &q2, const float& q3)
{
    /**
     * TODO: Construct forward kinematic solver object,
     * and solve end effector pose given the joint values
     */
    KDL::JntArray joint_positions(chain_.getNrOfJoints());
    joint_positions(0) = q1;
    joint_positions(1) = q2;
    joint_positions(2) = q3;
    
    KDL::ChainFkSolverPos_recursive fksolver(chain_);
    KDL::Frame F_result;
    fksolver.JntToCart(joint_positions,F_result);

    return F_result;
}

KDL::JntArray Kinematics::compute_ik_kdl(const float &q1_init, const float &q2_init, const float& q3_init, const KDL::Frame &target_pose)
{
    /**
     * TODO: Construct inverse kinematics solver object,
     * and solve joint positions given the desired pose
     * and initial joint values.
     */
    KDL::JntArray q_i(chain_.getNrOfJoints());
    q_i(0) = q1_init;
    q_i(1) = q2_init;
    q_i(2) = q3_init;

    KDL::JntArray q_out(chain_.getNrOfJoints());

    KDL::ChainIkSolverPos_LMA ik_solver(chain_);
    
    int result = ik_solver.CartToJnt(q_i, target_pose, q_out);

    return q_out;
}

KDL::Jacobian Kinematics::compute_jac_kdl(const float &q1, const float &q2, const float& q3,
                                          const int& segment_n)
{
    /**
     * TODO: Construct Jacobian solver object, and
     * solve the Jacobian given the joint values.
     */
    KDL::JntArray q_i(chain_.getNrOfJoints());
    q_i(0) = q1;
    q_i(1) = q2;
    q_i(2) = q3;

    KDL::ChainJntToJacSolver jacobian_solver(chain_);

    KDL::Jacobian jacobian(chain_.getNrOfJoints());

    //int result = jacobian_solver.JntToJac(q_i, jacobian, segment_n);
    int result = jacobian_solver.JntToJac(q_i, jacobian, 3);

    return jacobian;
}
