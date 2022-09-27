//
// Created by jy on 22. 7. 11.
//

#ifndef RAISIM_REMOVEMATRIX_H
#define RAISIM_REMOVEMATRIX_H

#include "Eigen/Eigen"

class RemoveMatrix
{

public:
    Eigen::MatrixXd removeRow(Eigen::MatrixXd matrix, unsigned int rowToRemove);
    Eigen::MatrixXd removeColumn(Eigen::MatrixXd matrix, unsigned int colToRemove);

};


#endif //RAISIM_REMOVEMATRIX_H
