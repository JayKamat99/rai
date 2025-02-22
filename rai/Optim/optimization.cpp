/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "optimization.h"

#include <math.h>

uint eval_count=0;

//===========================================================================

double Conv_NLP_ScalarProblem::scalar(arr& g, arr& H, const arr& x){
  arr phi, J;
  P->evaluate(phi, J, x);

  CHECK_EQ(phi.N, P->featureTypes.N, "");
  CHECK_EQ(phi.N, J.d0, "");
  CHECK_EQ(x.N, J.d1, "");

  double f=0.;
  for(uint i=0; i<phi.N; i++) {
    if(P->featureTypes.p[i]==OT_sos) f += rai::sqr(phi.p[i]);
    else if(P->featureTypes.p[i]==OT_f) f += phi.p[i];
    else HALT("this must be an unconstrained problem!")
  }

  if(!!g) { //gradient
    arr coeff=zeros(phi.N);
    for(uint i=0; i<phi.N; i++) {
      if(P->featureTypes.p[i]==OT_sos) coeff.p[i] += 2.* phi.p[i];
      else if(P->featureTypes.p[i]==OT_f) coeff.p[i] += 1.;
    }
    g = comp_At_x(J, coeff);
    g.reshape(x.N);
  }

  if(!!H) { //hessian: Most terms are of the form   "J^T  diag(coeffs)  J"
    arr coeff=zeros(phi.N);
    double hasF=false;
    for(uint i=0; i<phi.N; i++) {
      if(P->featureTypes.p[i]==OT_sos) coeff.p[i] += 2.;
      else if(P->featureTypes.p[i]==OT_f) hasF=true;
    }
    arr tmp = J;
    if(!isSparseMatrix(tmp)) {
      for(uint i=0; i<phi.N; i++) tmp[i] *= sqrt(coeff.p[i]);
    } else {
      arr sqrtCoeff = sqrt(coeff);
      tmp.sparse().rowWiseMult(sqrtCoeff);
    }
    H = comp_At_A(tmp); //Gauss-Newton type!

    if(hasF) { //For f-terms, the Hessian must be given explicitly, and is not \propto J^T J
      arr fH;
      P->getFHessian(fH, x);
      if(fH.N) H += fH;
    }

    if(!H.special) H.reshape(x.N, x.N);
  }

  return f;
}


//===========================================================================
//
// checks and converters
//

bool checkJacobianCP(NLP& P, const arr& x, double tolerance) {
  VectorFunction F = [&P](const arr& x) {
    arr phi, J;
    P.evaluate(phi, J, x);
    phi.J() = J;
    return phi;
  };
  return checkJacobian(F, x, tolerance);
}

bool checkHessianCP(NLP& P, const arr& x, double tolerance) {
  uint i;
  arr phi, J;
  P.evaluate(phi, NoArr, x); //TODO: only call getStructure
  for(i=0; i<P.featureTypes.N; i++) if(P.featureTypes(i)==OT_f) break;
  if(i==P.featureTypes.N) {
    RAI_MSG("no f-term in this KOM problem");
    return true;
  }
  ScalarFunction F = [&P, &phi, &J, i](arr& g, arr& H, const arr& x) -> double{
    P.evaluate(phi, J, x);
    P.getFHessian(H, x);
    g = J[i];
    return phi(i);
  };
  return checkHessian(F, x, tolerance);
}

void boundClip(NLP& P, arr& x){
  arr bounds_lo, bounds_up;
  P.getBounds(bounds_lo, bounds_up);
  boundClip(x, bounds_lo, bounds_up);
}

bool checkInBound(NLP& P, const arr& x){
  arr bound_lo, bound_up;
  P.getBounds(bound_lo, bound_up);
  CHECK_EQ(x.N, bound_lo.N, "");
  CHECK_EQ(x.N, bound_up.N, "");
  return boundCheck(x, bound_lo, bound_up);
}


//void OptOptions::write(std::ostream& os) const {
//#define WRT(x) os <<#x <<" = " <<x <<endl;
//  WRT(verbose);
////  double *fmin_return);
//  WRT(stopTolerance);
//  WRT(stopEvals);
//  WRT(stopIters);
//  WRT(initStep);
//  WRT(minStep);
//  WRT(maxStep);
//  WRT(damping);
//  WRT(stepInc);
//  WRT(stepDec);
//  WRT(dampingInc);
//  WRT(dampingDec);
//  WRT(nonStrictSteps);
//  WRT(allowOverstep);
//  WRT(constrainedMethod);
//  WRT(aulaMuInc);
//#undef WRT
//}

//===========================================================================
//
// helpers
//

void displayFunction(const ScalarFunction& f, bool wait, double lo, double hi) {
  arr X, Y;
  X.setGrid(2, lo, hi, 100);
  Y.resize(X.d0);
  for(uint i=0; i<X.d0; i++) {
    double fx=f(NoArr, NoArr, X[i]);
    Y(i) = ((fx==fx && fx<10.)? fx : 10.);
  }
  Y.reshape(101, 101);
  FILE("z.fct") <<(~Y).modRaw();
//  plot()->Gnuplot();  plot()->Surface(Y);  plot()->update(true);
//  write(LIST<arr>(Y), "z.fct");
  gnuplot("reset; set xlabel 'x'; set ylabel 'y'; splot [-1:1][-1:1] 'z.fct' matrix us ($1/50-1):($2/50-1):3 w l", wait, true);
}

/// minimizes \f$f(x)\f$ using its gradient only
uint optGradDescent(arr& x, const ScalarFunction& f, rai::OptOptions o) {
  uint evals=0;
  arr y, grad_x, grad_y;
  double fx, fy;
  double a=o.initStep;

  fx = f(grad_x, NoArr, x);  evals++;
  if(o.verbose>1) cout <<"*** optGradDescent: starting point x=" <<(x.N<20?x:arr()) <<" f(x)=" <<fx <<" a=" <<a <<endl;
  ofstream fil;
  if(o.verbose>0) fil.open("z.opt");
  if(o.verbose>0) fil <<0 <<' ' <<eval_count <<' ' <<fx <<' ' <<a <<' ' <<x <<endl;

  grad_x /= length(grad_x);

  for(uint k=0;; k++) {
    y = x - a*grad_x;
    fy = f(grad_y, NoArr, y);  evals++;
    CHECK_EQ(fy, fy, "cost seems to be NAN: fy=" <<fy);
    if(o.verbose>1) cout <<"optGradDescent " <<evals <<' ' <<eval_count <<" \tprobing y=" <<(y.N<20?y:arr()) <<" \tf(y)=" <<fy <<" \t|grad|=" <<length(grad_y) <<" \ta=" <<a;

    if(fy <= fx) {
      if(o.verbose>1) cout <<" - ACCEPT" <<endl;
      double step=length(x-y);
      x = y;
      fx = fy;
      grad_x = grad_y/length(grad_y);
      a *= 1.2;
      if(o.maxStep>0. && a>o.maxStep) a = o.maxStep;
      if(o.verbose>0) fil <<evals <<' ' <<eval_count <<' ' <<fx <<' ' <<a <<' ' <<x <<endl;
      if(step<o.stopTolerance) break;
    } else {
      if(o.verbose>1) cout <<" - reject" <<endl;
      a *= .5;
    }
    if(o.stopEvals>0 && evals>(uint)o.stopEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
    if(o.stopIters>0 && k>(uint)o.stopIters) break;
  }
  if(o.verbose>0) fil.close();
  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", true);
  return evals;
}

RUN_ON_INIT_BEGIN(optimization)
ObjectiveTypeA::memMove=true;
RUN_ON_INIT_END(optimization)
