
#include "cartographer/transform/rigid_transform.h"

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include <iostream>
namespace cartographer {
namespace transform {
#define sout(Xit)  {std::cout<<__LINE__<<" "<< Xit <<""<<std::endl;}
namespace {


TEST(Rigid2s, rigid_1) {
  Rigid2<double> r1; 
  sout(r1);                 //[1,0,0]
  sout(r1.Identity());      //[0,0,0]
  sout(r1.Rotation(3.14/2));//[0,0,1.57]

  sout(r1.Translation({1,2}));//[1,2,0],静态成员函数，与对象无关
  sout(Rigid2<int>::Translation({1,2}))//同上

  sout(r1.inverse());      //[-1,-0,-0]
  sout(r1.inverse().inverse());//[1,0,0]

  sout(r1*r1);              //[2,0,0]
  Eigen::Matrix<double, 2, 1> mat{1,2};
  sout(r1*mat);             //[2,2] ：[1,0]+[1,2]
  //sout((r1*mat)(1,0));//Eigen矩阵索引是()而不是[]
  
 }

TEST(Rigid2s, rigid_2) {
  Rigid2<double> r1({1,2},M_PI_2); 
  sout(M_PI_2)             //1.57
  sout(r1);                //[1,2,1.57] ，pi/2对应的弧度是1.57
  sout(r1.Identity());     //[0,0,0]
  sout(r1.Rotation(3.14/2));//[0,0,1.57]静态成员函数，与对象无关
  
  //sout(Rigid2<int>::Rotation(3.14/2));//同上
  sout(r1.Translation({1,2}));//[1,2,0]


  sout(r1);              //[1,2,1.57]
  sout(r1.inverse());        //[-2,1,-1.57]
  sout(r1.inverse().inverse());

  Rigid2<double> r2({2,2},M_PI_2); 
  sout(r2.cast<float>()); //[2.0,2.0,1.0],成员模板函数cast()
  

  Rigid2<double> r3({3,2},3.2*2);
  sout(r3);                    //[3,2,6.4]
  sout(r3.normalized_angle()); //0.117

  
}

TEST(Rigid2s, rigid_3) {

  Rigid2<double> r1({1,2},M_PI_2); 
  Rigid2<double> r2({2,2},M_PI_2); 
  Rigid2<double> r4({2,2},M_PI_2); 

  sout(r1);                     //[1,2,1.57]
  sout(r1*r1);                 //[-1,3,3.14]
  sout(r1*r2);                 //[-1,4,3.14]

  Eigen::Matrix<double, 2, 1> mat{1,1};
  Eigen::Matrix<double, 2, 1> matd{1,1};


  sout(r1.rotation().matrix ()); //[0,-1;1,0]
  sout(r1.rotation() * mat ); //[-1,1]
  sout(r1.translation());     //[1,2] 
  sout(mat);                  //[1,1]
  sout(r1*mat); //[0,3].  




  sout(r4.rotation().matrix (   ) );//[0,-1;1,0]
  sout(r4.rotation() * matd );      //[-1,1] 
  sout(r4.translation());           //[2,2]
  sout(matd); //[1,1]
  sout(r4*matd); //[1,3].    
  }
}

}  // namespace transform
}  // namespace cartographer
