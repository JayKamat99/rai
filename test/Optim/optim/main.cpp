#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <functional>
#include <Optim/NLP_Solver.h>
#include <Optim/lagrangian.h>

//===========================================================================

void TEST(Display) {
  std::shared_ptr<NLP> nlp = getBenchmarkFromCfg();

  NLP_Viewer(nlp).display();

  rai::wait();
}

//===========================================================================

void TEST(Solver) {
  std::shared_ptr<NLP> nlp = getBenchmarkFromCfg();

//  displayNLP(nlp);

//  arr x = nlp->getInitializationSample();
//  checkJacobianCP(*nlp, x, 1e-4);

  arr x_init = rai::getParameter<arr>("x_init", {});
  NLP_Solver S;

  rai::Enum<NLP_SolverID> sid (rai::getParameter<rai::String>("solver"));
  S.setSolver(sid);
  S.setProblem(nlp);
  if(x_init.N) S.setInitialization(x_init);
  S.solveStepping();

  arr path = catCol(S.getTrace_x(), S.getTrace_costs());
  FILE("z.path") <<path.modRaw();

  NLP_Viewer(nlp, S.P). display();
  // displayNLP(nlp, S.getTrace_x(), S.getTrace_costs());
//  gnuplot("load 'plt'", false, false);
  rai::wait();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

//  testDisplay();
  testSolver();

  return 0;
}
