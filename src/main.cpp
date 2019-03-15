#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpFeaturePoint.h>
#include <ecn_sensorbased/optim.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    PioneerCam robot;

    // gains
    // pose error gain
    double lv = .5;
    geometry_msgs::Pose2D target;

    int it = 0;
    vpColVector v(4);

    //start
    double radius = robot.radius(), base = robot.base(), wmax = robot.wmax(),alpha = 1, eps = 0.001;
    vpColVector r(2),b(4),d(8),x(4);
    vpMatrix Q(2,4),A(4,4),C(8,4);
    //end


    while(ros::ok())
    {
        it++;
        cout << "-------------" << endl;

        if(robot.ok())
        {
            // get robot and target positions to get position error
            target = robot.getTargetRelativePose();

            // linear velocity
            v[0] = lv*(target.x - .1);//v^star
            // angular velocity
            v[1] = 10*lv*(fmod(atan2(target.y, target.x)+M_PI, 2*M_PI) - M_PI);//w^star

//            cout << "v: " << v.t() << endl;


            //star
            vpColVector im_p,im_b;
            robot.getImagePoint(im_p);
            im_b = robot.getCamLimits();
            vpFeaturePoint s;
            s.buildFrom(im_p[0], im_p[1], 1);
            vpMatrix L = s.interaction( vpBasicFeature::FEATURE_ALL );
            vpMatrix Jc = L * robot.getCamJacobian();
            Q[0][0] = Q[1][1] = 1 ;
//            Q[2][2] = Q[3][3] = eps;
//            Q[2][2] = Q[3][3] = eps;
            r[0] = v[0];
            r[1] = v[1];
//            r[2] = 0;
//            r[3] = 0;
//            r[2] =
//            r[3] =
            A[0][0] = v[1];
            A[0][1] = -v[0];

//            cout << "the potision of robot is " <<im_p << endl;
//            cout << "the limit of image is " << im_b << endl;

//            cout /*<< "C is " << C*/
//                 << "Jc is "<< Jc
//                 << "J is" << robot.getCamJacobian();
            C[0][0]=1/radius;
            C[0][1]=-base/radius;
            C[1][0]=1/radius;
            C[1][1]=base/radius;
            C[2][0]=-1/radius;
            C[2][1]=base/radius;
            C[3][0]=-1/radius;
            C[3][1]=-base/radius;
            ecn::putAt(C, Jc, 4, 0);
            ecn::putAt(C, -Jc, 6, 0);
//            cout << "C is " << C << endl;

            d[0] = wmax;
            d[1] = wmax;
            d[2] = wmax;
            d[3] = wmax;



            vpColVector d4 = alpha * (im_b - im_p);
            vpColVector d6 = -alpha * (-im_b - im_p);
            ecn::putAt(d, d4, 4);
            ecn::putAt(d, d6, 6);


//            cout<<"\nwmax = \n"<<wmax<<endl;
            cout << "\nQ = \n" << Q << "\nr = \n" << r << "\nA = \n" << A << "\nb = \n" << b << "\nC = \n" << C << "\nd = \n" << d << "\nx = \n" <<x << endl;
            ecn::solveQP(Q,r,A,b,C,d,x);




            //end

            robot.setVelocity(x);
        }
    }
}
