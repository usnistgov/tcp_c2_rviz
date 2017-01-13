
/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */
//#pragma message "Compiling " __FILE__ 

#include "Conversions.h"

// Eigen conversions
// http://docs.ros.org/jade/api/eigen_conversions/html/eigen__msg_8cpp_source.html

namespace Conversion {

    //==================================================
    // Adapted from: https://github.com/davetcoleman/rviz_visual_tools/blob/kinetic-devel/src/rviz_visual_tools.cpp
    // http://docs.ros.org/kinetic/api/tf_conversions/html/c++/tf__eigen_8cpp_source.html
    template<typename T>
    static void transformEigenToTFImpl(const T &e, tf::Transform &t) {
        t.setOrigin(tf::Vector3(e.matrix()(0, 3), e.matrix()(1, 3), e.matrix()(2, 3)));
        t.setBasis(tf::Matrix3x3(e.matrix()(0, 0), e.matrix()(0, 1), e.matrix()(0, 2),
                e.matrix()(1, 0), e.matrix()(1, 1), e.matrix()(1, 2),
                e.matrix()(2, 0), e.matrix()(2, 1), e.matrix()(2, 2)));
    }

    template<typename Transform>
    void transformTFToEigenImpl(const tf::Transform &t, Transform & e) {
        for (int i = 0; i < 3; i++) {
            e.matrix()(i, 3) = t.getOrigin()[i];
            for (int j = 0; j < 3; j++) {
                e.matrix()(i, j) = t.getBasis()[i][j];
            }
        }
        // Fill in identity in last row
        for (int col = 0; col < 3; col++)
            e.matrix()(3, col) = 0;
        e.matrix()(3, 3) = 1;
    }
}