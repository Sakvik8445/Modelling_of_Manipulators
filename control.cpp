#include <robot_init.h>

using namespace std;
using namespace ecn;

int main(int argc, char ** argv)
{
    // initialize robot class and get DOF's
    const auto robot{initRobot(argc, argv, 100)};
    const unsigned n{robot->getDofs()};

    // robot properties - max velocity and acceleration
    const auto vMax{robot->vMax()};
    const auto aMax{robot->aMax()};

    // main variables
    vpColVector q(n);               // joint position
    vpPoseVector p;                 // operational pose
    vpColVector qCommand(n);        // joint position setpoint
    vpColVector vCommand(n);        // joint velocity setpoint

    vpMatrix J;
    vpHomogeneousMatrix M;          // current pose
    vpHomogeneousMatrix M0, Md, Mi; // previous, final and current desired poses
    vpPoseVector pd;                // desired pose
    vpColVector v;                  // desired operational velocity

    // TODO declare other variables if needed
    vpPoseVector pe;                // error pose
    vpColVector ve(6);              // final error velocity
    vpColVector q0(n), qf(n);       // joint position setpoint for initial and final poses
    double t0, tf;

    // main control loop
    while(robot->ok())
    {
        // current time
        const auto t{robot->time()};

        // update desired pose if has changed
        if(robot->newRef())
        {
            Md = robot->Md();
            M0 = robot->M0();
            pd.buildFrom(Md);
            t0 = t;
        }

        // get current joint positions
        q = robot->jointPosition();
        //cout << "Current joint position : " << q.t() << endl;

        // Direct Geometry for end-effector
        M = robot->fMe(q);  // matrix form
        p.buildFrom(M);     // translation + angle-axis form

        if(robot->mode() == ControlMode::POSITION_MANUAL)
        {
            // just check the Direct Geometric Model
            // DONE - TODO: fill the fMw function
            robot->checkPose(M);
        }


        else if(robot->mode() == ControlMode::VELOCITY_MANUAL)
        {
            // follow a given operational velocity
            v = robot->guiVelocityScrew();

            // DONE - TODO: fill the fJw function
            // DONE - TODO: compute vCommand
            vpRotationMatrix R = M.getRotationMatrix();
            vpMatrix temp (6,6);
            ecn::putAt(temp, R, 0, 0);
            ecn::putAt(temp, R, 3, 3);

            auto J_inv = robot->fJe(q).pseudoInverse();

            robot->setJointVelocity(vCommand);
        }


        else if(robot->mode() == ControlMode::DIRECT_P2P)
        {
            // find the Inverse Geometry to reach Md
            // DONE - TODO: fill the inverseGeometry function
            qf = robot->inverseGeometry(Md, q);
            robot->setJointPosition(qf);
        }

        else if(robot->mode() == ControlMode::POLYNOM_P2P) //Exercise 4: joint coordinates
        {
            // reach Md with polynomial joint trajectory
            // use q0 (initial position), qf (final), aMax and vMax

            // if reference has changed, compute new tf
            if(robot->newRef())
            {
                q0 = robot->inverseGeometry(M0, q);
                qf = robot->inverseGeometry(Md, q);
                tf = 0;
                for (unsigned int i = 0; i < n; i++)
                {
                    auto dq = qf[i] - q0[i];
                    auto tfv = 3 * std::abs(dq) / (2 * vMax[i]);    //use contraints to find smallest tf
                    auto tfa = sqrt(6 * std::abs(dq) / aMax[i]);
                    auto tfmax{0};

                    if (tfv > tfa)
                    {
                        tfmax = tfv;
                    }
                    else
                    {
                        tfmax = tfa;
                    }
                    if (tfmax > tf)
                    {
                        tf = tfmax;
                    }
                }


            }

            // DONE - TODO: compute qCommand from q0, qf, t, t0 and tf
            auto dq = qf - q0;      auto dt = t - t0;
            auto Pt = 3 * pow(dt / tf, 2) - 2 * pow(dt / tf, 3);
            qCommand = q0 + Pt * (dq);

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ControlMode::STRAIGHT_LINE_P2P) //Exercise 5: cartesian coordinates
        {
            // go from M0 to Md in 1 sec
            tf = 1;
            auto dt = t - t0;
            auto alpha{1 * (dt > tf)};
            if (alpha == 0)
            {
                alpha = dt / tf;
            }
            // DONE - TODO: compute qCommand from M0, Md, t, t0 and tf
            // use robot->intermediaryPose to build poses between M0 and Md
            auto pose = robot->intermediaryPose(M0, Md, alpha); //cartescian, intermediate poses position between M0 and Md, alpha grows
            qCommand = robot->inverseGeometry(pose, q); // use IGM to transform it into joint space

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ControlMode::VELOCITY_P2P)
        {
            // go to Md using operational velocity
            auto Me = Md.inverse() * M; //Me error homogenuous matrix of final frame wrt intermediate desired frame
            pe.buildFrom(Me);  // contains the pose data in t & thetau form
            auto eThetaU = pe.getThetaUVector();
            auto eT = pe.getTranslationVector(); //decompose Me into translation and rotation
            auto lambda = - robot->lambda();
            auto fRe_star = Md.getRotationMatrix();

            auto fRe = M.getRotationMatrix();

            auto v = lambda * fRe_star * eT;
            auto w = lambda * fRe * eThetaU.getU() * eThetaU.getTheta();
            std::cout << "v : " << v << std::endl;
            std::cout << "w : " << w << std::endl;

            // DONE - TODO: compute joint velocity command

            for (int i = 0; i < 6; i++)
            {
                if (i < 3)
                {
                    ve[i] = v[i];
                }
                else
                {
                    ve[i] = w[i-3];
                }
            }
            vCommand = robot->fJe(q).pseudoInverse() * ve;
            robot->setJointVelocity(vCommand);
        }
    }
}
