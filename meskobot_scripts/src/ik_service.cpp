#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>
#include "meskobot_scripts/srv/sol.hpp"

using namespace std;
#define PI 3.14159265358979323846


struct DHParameter {
    double a;
    double alpha;
    double d;
    double theta;
};


struct Joint_Pose
{
    double Theta_1;
    double Theta_2;
    double Theta_3;
    double Theta_4;
    double Theta_5;
    double Theta_6;
};


using Vector3 = std::array<double, 3>;
using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;


class IKClass : public rclcpp::Node
{
public:
    IKClass() : Node("ik_service_node"){
        service_obj = this->create_service<meskobot_scripts::srv::Sol>(
        "ik_compute", std::bind(&IKClass::ik_func, this,std::placeholders::_1, std::placeholders::_2));

    }


    Matrix multiplyMatrices(const Matrix& A, const Matrix& B) {
        Matrix result(A.size(), std::vector<double>(B[0].size(), 0.0));
        for (size_t i = 0; i < A.size(); ++i) {
            for (size_t j = 0; j < B[0].size(); ++j) {
                for (size_t k = 0; k < A[0].size(); ++k) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }


    Matrix dhTransform(double a, double alpha, double d, double theta) {
        Matrix T =  {{
            {cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta)},
            {sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)},
            {0,           sin(alpha),               cos(alpha),               d},
            {0,           0,                        0,                        1}
        }};
        return T;
    }

    Matrix Inverse_Matrix(Matrix&T){
        Matrix T_inv = {{
            {1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        }};
        Eigen::MatrixXd ans(4,4);
        for(int i =0; i<4;i++){
            for(int j=0;j<4;j++){
                ans(i,j)=T[i][j];
            }
        }
        // Code for pseudo-inverse calculation 
        Eigen::MatrixXd inverse = ans.inverse();
        for(int i =0; i<4;i++){
            for(int j=0;j<4;j++){
                T_inv[i][j]=inverse(i,j);
            }
        }
        return T_inv;
    }


    std::vector<Joint_Pose>  Inverse_position_kinematics(Matrix T){

        //****************/ robot parameters**********

        double d1 = 0.1454;
        double d2 = 0;
        double d3 = 0;
        double d4 = 0.0405+0.0352+0.005;
        double d5 = 0.08;
        double d6 = 0.03;

        double a2 = 0.147;
        double a3 = 0.147;


        Vector3 p6 = {T[0][3],T[1][3],T[2][3]}; // COORDINATES OF EE
        Vector3 z6 = {T[0][2],T[1][2],T[2][2]}; //DIRECTION OF AXIS Z6 OF EE
        Vector3 x6 = {T[0][0],T[1][0],T[2][0]}; //DIRECTION OF AXIS Z6 OF EE
        Vector3 p5 = {T[0][3]-(d6*z6[0]),T[1][3]-(d6*z6[1]),T[2][3]-(d6*z6[2])}; // COORDINATES OF FRAME 5

        // ************Calculating  th1**************

        // ************While atan2(y,x) has range (-pi,pi) and takes into account sign of x,y!******************

        double alpha = atan2(p5[1],p5[0]);

        double beta = (d2+d3+d4)/sqrt(p5[1]*p5[1] + p5[0]*p5[0]);

        double th11 = alpha + asin(beta);
        double th12 = PI + alpha - asin(beta); 


        // *********CALCULATING TH5***********
        double j51 = p6[0]*sin(th11) - p6[1]*cos(th11);
        double j52 = p6[0]*sin(th12) - p6[1]*cos(th12);
        double w51 = (j51 - d2 - d3 - d4)/d6;
        double w52 = (j52 - d2 - d3 - d4)/d6;
   
        double th51 = acos(w51);
        double th52 = -acos(w51);
        double th53 = acos(w52);
        double th54 = -acos(w52);


        Matrix  T0_1 = dhTransform(0,PI/2,0.1454,th11);
        Matrix  T0_2 = dhTransform(0,PI/2,0.1454,th12);

        Matrix T1_01 = Inverse_Matrix(T0_1);
        Matrix T1_02 = Inverse_Matrix(T0_2);

        Matrix T1_61 = multiplyMatrices(T1_01,T);
        Matrix T1_62 = multiplyMatrices(T1_02,T);

        Matrix T6_11 = Inverse_Matrix(T1_61);
        Matrix T6_12 = Inverse_Matrix(T1_62);

        double p611 = T6_11[0][2]/sin(th51); //Zx
        double p612 = T6_11[0][2]/sin(th52);
        double p621 = T6_12[0][2]/sin(th53);
        double p622 = T6_12[0][2]/sin(th54);

        double q611 = T6_11[1][2]/sin(th51); //Zy
        double q612 = T6_11[1][2]/sin(th52);
        double q621 = T6_12[1][2]/sin(th53);
        double q622 = T6_12[1][2]/sin(th54);

        double th61 = atan2(-q611,p611);
        double th62 =  atan2(-q612,p612);
        double th63 = atan2(-q621,p621);
        double th64 = atan2(-q622,p622);


        Matrix T4_51 = dhTransform(0, -PI/2, 0.08, th51);
        Matrix T4_52 = dhTransform(0, -PI/2, 0.08, th52);
        Matrix T4_53 = dhTransform(0, -PI/2, 0.08, th53);
        Matrix T4_54 = dhTransform(0, -PI/2, 0.08, th54);

        Matrix T5_61 = dhTransform(0, 0, 0.03, th61);
        Matrix T5_62 = dhTransform(0, 0, 0.03, th62);
        Matrix T5_63 = dhTransform(0, 0, 0.03, th63);
        Matrix T5_64 = dhTransform(0, 0, 0.03, th64);

        Matrix T4_61 = multiplyMatrices(T4_51, T5_61);
        Matrix T4_62 = multiplyMatrices(T4_52, T5_62);
        Matrix T4_63 = multiplyMatrices(T4_53, T5_63);
        Matrix T4_64 = multiplyMatrices(T4_54, T5_64);

        Matrix T6_41 = Inverse_Matrix(T4_61);
        Matrix T6_42 = Inverse_Matrix(T4_62);
        Matrix T6_43 = Inverse_Matrix(T4_63);
        Matrix T6_44 = Inverse_Matrix(T4_64);

        Matrix T1_41 = multiplyMatrices(T1_61, T6_41);
        Matrix T1_42 = multiplyMatrices(T1_61, T6_42);
        Matrix T1_43 = multiplyMatrices(T1_62, T6_43);
        Matrix T1_44 = multiplyMatrices(T1_61, T6_44);

        Vector3 p31 = {T1_41[0][3] - d4*T1_41[0][1], T1_41[1][3] - d4*T1_41[1][1], T1_41[2][3] - d4*T1_41[2][1]};
        Vector3 p32 = {T1_42[0][3] - d4*T1_42[0][1], T1_42[1][3] - d4*T1_42[1][1], T1_42[2][3] - d4*T1_42[2][1]};
        Vector3 p33 = {T1_43[0][3] - d4*T1_43[0][1], T1_43[1][3] - d4*T1_43[1][1], T1_43[2][3] - d4*T1_43[2][1]};
        Vector3 p34 = {T1_44[0][3] - d4*T1_44[0][1], T1_44[1][3] - d4*T1_44[1][1], T1_44[2][3] - d4*T1_44[2][1]};


        double val = 2*a2*a3 + a2*a2 + a3*a3;
                
        double j31 = (-(a2*a2 + a3*a3) + (p31[0]*p31[0] + p31[1]*p31[1] + p31[2]*p31[2])) / (2*(a2*a3));
        double j32 = (-(a2*a2 + a3*a3) + (p32[0]*p32[0] + p32[1]*p32[1] + p32[2]*p32[2])) / (2*(a2*a3));
        double j33 = (-(a2*a2 + a3*a3) + (p33[0]*p33[0] + p33[1]*p33[1] + p33[2]*p33[2])) / (2*(a2*a3));
        double j34 = (-(a2*a2 + a3*a3) + (p34[0]*p34[0] + p34[1]*p34[1] + p34[2]*p34[2])) / (2*(a2*a3));

        double th31 = acos(j31);
        double th32 = acos(j32);
        double th33 = acos(j33);
        double th34 = acos(j34);
        double th35 = -acos(j31);
        double th36 = -acos(j32);
        double th37 = -acos(j33);
        double th38 = -acos(j34);


        double th21 = atan2(p31[1], p31[0]) - asin((a3 * sin(th31)) / sqrt(p31[0]*p31[0] + p31[1]*p31[1] + p31[2]*p31[2]));
        double th22 = atan2(p32[1], p32[0]) - asin((a3 * sin(th32)) / sqrt(p32[0]*p32[0] + p32[1]*p32[1] + p32[2]*p32[2]));
        double th23 = atan2(p33[1], p33[0]) - asin((a3 * sin(th33)) / sqrt(p33[0]*p33[0] + p33[1]*p33[1] + p33[2]*p33[2]));
        double th24 = atan2(p34[1], p34[0]) - asin((a3 * sin(th34)) / sqrt(p34[0]*p34[0] + p34[1]*p34[1] + p34[2]*p34[2]));
        double th25 = atan2(p31[1], p31[0]) - asin((a3 * sin(th35)) / sqrt(p31[0]*p31[0] + p31[1]*p31[1] + p31[2]*p31[2]));
        double th26 = atan2(p32[1], p32[0]) - asin((a3 * sin(th36)) / sqrt(p32[0]*p32[0] + p32[1]*p32[1] + p32[2]*p32[2]));
        double th27 = atan2(p33[1], p33[0]) - asin((a3 * sin(th33)) / sqrt(p33[0]*p33[0] + p33[1]*p33[1] + p33[2]*p33[2]));
        double th28 = atan2(p34[1], p34[0]) - asin((a3 * sin(th34)) / sqrt(p34[0]*p34[0] + p34[1]*p34[1] + p34[2]*p34[2]));

        if(th31<-2*PI){
            th21 = 2*PI + th21;
        }

        if(th32<-2*PI){
            th22 = 2*PI + th22;
        }

        if(th33<-2*PI){
            th23 = 2*PI + th23;
        }

        if(th34<-2*PI){
            th24 = 2*PI + th24;
        }
        if(th35<-2*PI){
            th25 = 2*PI + th25;
        }
        if(th36<-2*PI){
            th26 = 2*PI + th26;
        }
        if(th37<-2*PI){
            th27 = 2*PI + th27;
        }
        if(th38<-2*PI){
            th28 = 2*PI + th28;
        }



        Matrix T1_21 = dhTransform(0.147, 0, 0, th21);
        Matrix T1_22 = dhTransform(0.147, 0, 0, th22);
        Matrix T1_23 = dhTransform(0.147, 0, 0, th23);
        Matrix T1_24 = dhTransform(0.147, 0, 0, th24);
        Matrix T1_25 = dhTransform(0.147, 0, 0, th25);
        Matrix T1_26 = dhTransform(0.147, 0, 0, th26);
        Matrix T1_27 = dhTransform(0.147, 0, 0, th27);
        Matrix T1_28 = dhTransform(0.147, 0, 0, th28);

        Matrix T2_31 = dhTransform(0.147, 0, 0, th31);
        Matrix T2_32 = dhTransform(0.147, 0, 0, th32);
        Matrix T2_33 = dhTransform(0.147, 0, 0, th33);
        Matrix T2_34 = dhTransform(0.147, 0, 0, th34);
        Matrix T2_35 = dhTransform(0.147, 0, 0, th35);
        Matrix T2_36 = dhTransform(0.147, 0, 0, th36);
        Matrix T2_37 = dhTransform(0.147, 0, 0, th37);
        Matrix T2_38 = dhTransform(0.147, 0, 0, th38);

        Matrix T1_31 = multiplyMatrices(T1_21, T2_31);
        Matrix T1_32 = multiplyMatrices(T1_22, T2_32);
        Matrix T1_33 = multiplyMatrices(T1_23, T2_33);
        Matrix T1_34 = multiplyMatrices(T1_24, T2_34);
        Matrix T1_35 = multiplyMatrices(T1_25, T2_35);
        Matrix T1_36 = multiplyMatrices(T1_26, T2_36);
        Matrix T1_37 = multiplyMatrices(T1_27, T2_37);
        Matrix T1_38 = multiplyMatrices(T1_28, T2_38);

        Matrix T3_11 = Inverse_Matrix(T1_31);
        Matrix T3_12 = Inverse_Matrix(T1_32);
        Matrix T3_13 = Inverse_Matrix(T1_33);
        Matrix T3_14 = Inverse_Matrix(T1_34);
        Matrix T3_15 = Inverse_Matrix(T1_35);
        Matrix T3_16 = Inverse_Matrix(T1_36);
        Matrix T3_17 = Inverse_Matrix(T1_37);
        Matrix T3_18 = Inverse_Matrix(T1_38);

        Matrix T3_41 = multiplyMatrices(T3_11,T1_41);
        Matrix T3_42 = multiplyMatrices(T3_12,T1_42);
        Matrix T3_43 = multiplyMatrices(T3_13,T1_43);
        Matrix T3_44 = multiplyMatrices(T3_14,T1_44);
        Matrix T3_45 = multiplyMatrices(T3_15,T1_41);
        Matrix T3_46 = multiplyMatrices(T3_16,T1_42);
        Matrix T3_47 = multiplyMatrices(T3_17,T1_43);
        Matrix T3_48 = multiplyMatrices(T3_18,T1_44);

        double th41 = atan2(T3_41[1][0], T3_41[0][0]);
        double th42 = atan2(T3_42[1][0], T3_42[0][0]);
        double th43 = atan2(T3_43[1][0], T3_43[0][0]);
        double th44 = atan2(T3_44[1][0], T3_44[0][0]);
        double th45 = atan2(T3_45[1][0], T3_45[0][0]);
        double th46 = atan2(T3_46[1][0], T3_46[0][0]);
        double th47 = atan2(T3_47[1][0], T3_47[0][0]);
        double th48 = atan2(T3_48[1][0], T3_48[0][0]);


    // ****************THE 8 SOLUTION SET*******************
        Joint_Pose jpose1 = {th11,th51,th61,th31,th21,th41};
        Joint_Pose jpose2 = {th11,th51,th61,th35,th25,th45};
        Joint_Pose jpose3 = {th11,th52,th62,th32,th22,th42};
        Joint_Pose jpose4 = {th11,th52,th62,th36,th26,th46};
        Joint_Pose jpose5 = {th12,th53,th63,th33,th23,th43};
        Joint_Pose jpose6 = {th12,th53,th63,th37,th27,th47};
        Joint_Pose jpose7 = {th12,th54,th64,th34,th24,th44};
        Joint_Pose jpose8 = {th12,th54,th64,th38,th28,th48};

        std::vector<Joint_Pose> ik_response={jpose1 ,jpose2 ,jpose3 ,jpose4 ,jpose5 ,jpose6 ,jpose7 ,jpose8 } ;
        return ik_response;
        
    }

      
    Matrix Transform_matrix(std::vector<double>& ik_request){


        double q0 = ik_request[6];
        double q1 = ik_request[3];
        double q2 = ik_request[4];
        double q3 = ik_request[5];

        double r00 = 2 * (q0 * q0 + q1 * q1) - 1;
        double r01 = 2 * (q1 * q2 - q0 * q3);
        double r02 = 2 * (q1 * q3 + q0 * q2);

        double r10 = 2 * (q1 * q2 + q0 * q3);
        double r11 = 2 * (q0 * q0 + q2 * q2) - 1;
        double r12 = 2 * (q2 * q3 - q0 * q1);

        double r20 = 2 * (q1 * q3 - q0 * q2) ;   
        double r21 = 2 * (q2 * q3 + q0 * q1)  ;  
        double r22 = 2 * (q0 * q0 + q3 * q3) - 1;
    
        Matrix result = {{
            {r00, r01, r02, ik_request[0]},
            {r10, r11, r12, ik_request[1]},
            {r20, r21, r22, ik_request[2]},
            {0, 0, 0, 1}
        }};

        return result;

    }




private:
    rclcpp::Service<meskobot_scripts::srv::Sol>::SharedPtr service_obj;

    void ik_func(const std::shared_ptr<meskobot_scripts::srv::Sol::Request> request,
      const std::shared_ptr<meskobot_scripts::srv::Sol::Response> response)
      { 
        std::vector<double> ik_request ={request->ik_pos[0],request->ik_pos[1],request->ik_pos[2],request->ik_pos[3],request->ik_pos[4],request->ik_pos[5],request->ik_pos[6]};

        Matrix T = Transform_matrix(ik_request);        
        
        std::vector<Joint_Pose> ik_response = Inverse_position_kinematics(T);

        response->sol1 ={ik_response[0].Theta_1,ik_response[0].Theta_2,ik_response[0].Theta_3,ik_response[0].Theta_4,ik_response[0].Theta_5,ik_response[0].Theta_6};
        response->sol2 ={ik_response[1].Theta_1,ik_response[1].Theta_2,ik_response[1].Theta_3,ik_response[1].Theta_4,ik_response[1].Theta_5,ik_response[1].Theta_6};
        response->sol3 ={ik_response[2].Theta_1,ik_response[2].Theta_2,ik_response[2].Theta_3,ik_response[2].Theta_4,ik_response[2].Theta_5,ik_response[2].Theta_6};
        response->sol4 ={ik_response[3].Theta_1,ik_response[3].Theta_2,ik_response[3].Theta_3,ik_response[3].Theta_4,ik_response[3].Theta_5,ik_response[3].Theta_6};
        response->sol5 ={ik_response[4].Theta_1,ik_response[4].Theta_2,ik_response[4].Theta_3,ik_response[4].Theta_4,ik_response[4].Theta_5,ik_response[4].Theta_6};
        response->sol6 ={ik_response[5].Theta_1,ik_response[5].Theta_2,ik_response[5].Theta_3,ik_response[5].Theta_4,ik_response[5].Theta_5,ik_response[5].Theta_6};
        response->sol7 ={ik_response[6].Theta_1,ik_response[6].Theta_2,ik_response[6].Theta_3,ik_response[6].Theta_4,ik_response[6].Theta_5,ik_response[6].Theta_6};
        response->sol8 ={ik_response[7].Theta_1,ik_response[7].Theta_2,ik_response[7].Theta_3,ik_response[7].Theta_4,ik_response[7].Theta_5,ik_response[7].Theta_6};

      }

};

int main(int argc, char *argv[])
{
  // Initialize ROS2.
  rclcpp::init(argc, argv);

  // Create the node and spin it.
  rclcpp::spin(std::make_shared<IKClass>());

  // Shutdown ROS2.
  rclcpp::shutdown();
  return 0;
}
