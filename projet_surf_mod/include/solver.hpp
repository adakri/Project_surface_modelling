#ifndef SOLVER_H
#define SOLVER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>


//using namespace Eigen;


class Solver
{
public:
  Solver();
  virtual ~Solver();
  virtual void setSystemMatrix(Eigen::SparseMatrix<double,Eigen::RowMajor> systemMatrix) = 0;
  virtual Eigen::SparseVector<double> solve(Eigen::SparseVector<double> RHS) = 0;
};

class lsq_solver : public Solver
{
private:
  Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double>>  _solver;
  Eigen::SparseMatrix<double> _A;
  Eigen::VectorXd _rhs;
  Eigen::VectorXd _solution;
public:
  void setSystemMatrix(Eigen::SparseMatrix<double,Eigen::RowMajor> systemMatrix);
  void set_rhs(Eigen::VectorXd rhs);
  Eigen::SparseVector<double> solve();
};



#endif SOLVER_H
