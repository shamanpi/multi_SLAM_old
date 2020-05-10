#include "hw2.h"
#include <Eigen/Eigen>
#include <iostream> 
#include <thread>   





void EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
                  Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {
  // TODO


  // Note that these we passed by reference, so to return, just set them
Eigen::VectorXd b(6);
b<<u[0]*cos(x_hat_t[2])*dt,u[0]*sin(x_hat_t[2])*dt,u[1]*dt,u[2]*cos(x_hat_t[5])*dt,u[2]*sin(x_hat_t[5])*dt,u[3]*dt;
int l = x_hat_t.size();
Eigen::VectorXd v = Eigen::VectorXd::Zero(l); 
v.head(6) = b;
x_hat_tpdt = x_hat_t + v;

Eigen::Matrix3d Aa(3,3);
Eigen::Matrix3d Ab(3,3);

Eigen::MatrixXd A(6,6);
A = Eigen::MatrixXd::Zero(6,6);
Aa<<1,0,-dt*u[0]*sin(x_hat_t[2]),
0,1,dt*u[0]*cos(x_hat_t[2]),
0,0,1;

Ab<<1,0,-dt*u[2]*sin(x_hat_t[5]),
0,1,dt*u[2]*cos(x_hat_t[5]),
0,0,1;
//std::cout<<A.row(1);
A.block<3,3>(0,0) = Aa;
A.block<3,3>(3,3)= Ab;

Eigen::MatrixXd N(6,4);
N<<dt*u[0]*cos(x_hat_t[2]),0,0,0,
dt*u[0]*sin(x_hat_t[2]),0,0,0,
0,dt,0,0,
0,0,dt*u[2]*cos(x_hat_t[5]),0,
0,0,dt*u[2]*sin(x_hat_t[5]),0,
0,0,0,dt;
  Sigma_x_tpdt = Sigma_x_t;
  Sigma_x_tpdt.block<6,6>(0,0) = A*Sigma_x_t.block<6,6>(0,0)*(A.transpose()) + N*Sigma_n*(N.transpose());

for (int i = 6; i < l; i = i+2)
{
Sigma_x_tpdt.block<2,6>(i,0) = Sigma_x_t.block<2,6>(i,0)*(A.transpose());
Sigma_x_tpdt.block<6,2>(0,i) = A*Sigma_x_t.block<6,2>(0,i);
 
}
}


void EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, std::vector<Eigen::VectorXd> zs, std::vector<Eigen::MatrixXd> Sigma_ms,
                         Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt, int peram) {
  // TODO

  // For each measurement, check if it matches any already in the state, and run an update for it.

  // For every unmatched measurement make sure it's sufficiently novel, then add to the state.

  // Note that these we passed by reference, so to return, just set them

x_hat_tpdt = x_hat_t;
Sigma_x_tpdt = Sigma_x_t;
int l = x_hat_t.size();
int n = zs.size();
Eigen::MatrixXd HR1 = Eigen::MatrixXd::Zero(2,6);
Eigen::MatrixXd HL1(2,2);
Eigen::MatrixXd Hr(2,3);
Eigen::VectorXd G_p_L0;
Eigen::Vector3d Robpos;


if(peram == 0){
Hr<<-cos(x_hat_t[2]),-sin(x_hat_t[2]),-sin(x_hat_t[2])*(- x_hat_t[0])+cos(x_hat_t[2])*(- x_hat_t[1]),
sin(x_hat_t[2]),-cos(x_hat_t[2]),-cos(x_hat_t[2])*(- x_hat_t[0])-sin(x_hat_t[2])*(- x_hat_t[1]);




HL1<<cos(x_hat_t[2]),sin(x_hat_t[2]),
-sin(x_hat_t[2]),cos(x_hat_t[2]);

HR1.block<2,3>(0,0) = Hr;

G_p_L0 = x_hat_tpdt.head(2);
Robpos = x_hat_tpdt.head(3); 
}//ifends

else{


Hr<<-cos(x_hat_t[5]),-sin(x_hat_t[5]),-sin(x_hat_t[5])*(- x_hat_t[3])+cos(x_hat_t[5])*(- x_hat_t[4]),
sin(x_hat_t[5]),-cos(x_hat_t[5]),-cos(x_hat_t[5])*(- x_hat_t[3])-sin(x_hat_t[5])*(- x_hat_t[4]);




HL1<<cos(x_hat_t[5]),sin(x_hat_t[5]),
-sin(x_hat_t[5]),cos(x_hat_t[5]);

HR1.block<2,3>(0,3) = Hr;
G_p_L0 = x_hat_tpdt.segment<2>(3);
Robpos = x_hat_tpdt.segment<3>(3); 


}


if(l==6)
{





for (int i=0;i<n;i++)
{

int k = x_hat_tpdt.size();
Eigen::VectorXd G_p_L1 = HL1.inverse()*zs[i] + G_p_L0;

Eigen::VectorXd vec_joined(x_hat_tpdt.size() + G_p_L1.size());
vec_joined << x_hat_tpdt,G_p_L1 ;
x_hat_tpdt = vec_joined;


int m = x_hat_tpdt.size();

Eigen::MatrixXd newcov = Eigen::MatrixXd::Ones(m,m);
newcov.block(0,0,k,k) = Sigma_x_tpdt;


newcov.block<6,2>(0,k) = -Sigma_x_tpdt.block<6,6>(0,0)*(HR1.transpose())*(HL1.inverse().transpose());
newcov.block<2,6>(k,0) = -(HL1.inverse())*HR1*Sigma_x_tpdt.block<6,6>(0,0);
newcov.bottomRightCorner(2,2) = HL1.transpose()*(HR1*Sigma_x_tpdt.block<6,6>(0,0)*HR1.transpose() + Sigma_ms[i])*HL1;
if(m>8){
for (int j=6;j<m-2;j=j+2)
{

newcov.block<2,2>(j,k) = -Sigma_x_tpdt.block<2,6>(j,0)*(HR1.transpose())*(HL1);
newcov.block<2,2>(k,j) =-HL1.inverse()*HR1*Sigma_x_tpdt.block<6,2>(0,j);
}//forloop end/*/
}//ifend
Sigma_x_tpdt = newcov;
//std::cout<<Sigma_x_tpdt<<std::endl;
}//loop end


}//if end
else{
for(int i=0;i<n;i++){
int k = x_hat_tpdt.size();
Eigen::VectorXd r((k-6)/2);
r = Eigen::VectorXd::Zero((k-6)/2);


for(int j=6;j<k;j=j+2){

Eigen::Vector2d G_p_L(x_hat_tpdt[j],x_hat_tpdt[j+1]);
Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,k);

Eigen::MatrixXd HR(2,3);
HR<<-cos(Robpos[2]),-sin(Robpos[2]),-sin(Robpos[2])*(G_p_L[0] - Robpos[0])+cos(Robpos[2])*(G_p_L[1] - Robpos[1]),
sin(Robpos[2]),-cos(Robpos[2]),-cos(Robpos[2])*(G_p_L[0] - Robpos[0])-sin(Robpos[2])*(G_p_L[1] - Robpos[1]);

if(peram == 0){
H.block<2,3>(0,0) = HR;
}//ifend
else{H.block<2,3>(0,3) = HR;}//elseend


H.block<2,2>(0,j) = HL1;



Eigen::MatrixXd S;

S = H*Sigma_x_tpdt*(H.transpose()) + Sigma_ms[i];
//std::cout<<S.determinant()<<"\n";
Eigen::MatrixXd K = Sigma_x_tpdt*H.transpose()*S.inverse();

Eigen::VectorXd zcap(2);


zcap = HL1*(G_p_L.head(2)-Robpos.head(2));





double a = sqrt((zs[i]- zcap).transpose()*S.inverse()*(zs[i]-zcap));

r((j-6)/2) = sqrt((zs[i]- zcap).transpose()*S.inverse()*(zs[i]-zcap));
//std::cout<<"q"<<sqrt((zs[i]- zcap).transpose()*S.inverse()*(zs[i]-zcap))<<std::endl;
}//for j end

std::ptrdiff_t p;
float q = r.minCoeff(&p); 
std::cout<<q<<"\n";
//std::cout<< r.size() <<"  " << p <<"  " << r.minCoeff() << std::endl;
if(q>100){

continue;}//ifend
if(q>5){ 

//std::this_thread::sleep_for(std::chrono::milliseconds(10));
//std::cout<< r.size() <<"  " << p <<"  " << r.minCoeff() << std::endl;


Eigen::VectorXd G_p_L1 = HL1.inverse()*zs[i] + Robpos.head(2);

Eigen::VectorXd vec_joined(x_hat_tpdt.size() + G_p_L1.size());
vec_joined << x_hat_tpdt,G_p_L1 ;
x_hat_tpdt = vec_joined;

int m = x_hat_tpdt.size();

Eigen::MatrixXd newcov = Eigen::MatrixXd::Zero(m,m);
newcov.block(0,0,k,k) = Sigma_x_tpdt;


newcov.block<6,2>(0,k) = -Sigma_x_tpdt.block<6,6>(0,0)*(HR1.transpose())*(HL1.inverse().transpose());
newcov.block<2,6>(k,0) = -(HL1.inverse())*HR1*Sigma_x_tpdt.block<6,6>(0,0);
newcov.bottomRightCorner(2,2) = HL1.transpose()*(HR1*Sigma_x_tpdt.block<6,6>(0,0)*HR1.transpose() + Sigma_ms[i])*HL1;
if(m>8){
for (int j=6;j<m-2;j=j+2)
{
newcov.block<2,2>(j,k) = -Sigma_x_tpdt.block< 2,6>(j,0)*(HR1.transpose())*(HL1);
newcov.block<2,2>(k,j) =-HL1.inverse()*HR1*Sigma_x_tpdt.block<6,2>(0,j);
}//forloop end/*/
}//ifend
Sigma_x_tpdt = newcov;
}//ifend
else{
Eigen::Vector2d G_p_L(x_hat_tpdt[6+2*p],x_hat_tpdt[7+2*p]);
Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,k);

Eigen::MatrixXd HR(2,3);
HR<<-cos(Robpos[2]),-sin(Robpos[2]),-sin(Robpos[2])*(G_p_L[0] - Robpos[0])+cos(Robpos[2])*(G_p_L[1] - Robpos[1]),
sin(Robpos[2]),-cos(Robpos[2]),-cos(Robpos[2])*(G_p_L[0] - Robpos[0])-sin(Robpos[2])*(G_p_L[1] - Robpos[1]);

if(peram == 0){
H.block<2,3>(0,0) = HR;}//ifend
else{H.block<2,3>(0,3) = HR;}//elseend


//Eigen::MatrixXd HL1(2,2);



H.block<2,2>(0,6+2*p) = HL1;
Eigen::MatrixXd S;
S = H*Sigma_x_tpdt*(H.transpose()) + Sigma_ms[i];

Eigen::MatrixXd K;
K = Sigma_x_tpdt*H.transpose()*S.inverse();

Eigen::VectorXd zcap;
zcap = HL1*(G_p_L.head(2)-Robpos.head(2));
x_hat_tpdt = x_hat_tpdt + K*(zs[i]-zcap);
Eigen::MatrixXd I = Eigen::MatrixXd::Identity(k,k);
Sigma_x_tpdt = (I-K*H)*Sigma_x_tpdt*(I-K*H).transpose() + K*Sigma_ms[i]*K.transpose();

}//else end
}//for n end



  
}//elseend
//std::cout<<x_hat_tpdt.size()<< std::endl << "2nd line";

}
