/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "spline.h"
#include "../Core/util.h"

#include <math.h>

//==============================================================================
//
// Spline
//

namespace rai {

void Spline::clear() {
  points.clear();
  times.clear();
  knotPoints.clear();
  knotTimes.clear();
}

arr Spline::getCoeffs(double t, uint K, uint derivative) const {
  arr b(K+1), b_0(K+1), db(K+1), db_0(K+1), ddb(K+1), ddb_0(K+1);
  for(uint p=0; p<=degree; p++) {
    b_0=b; b.setZero();
    if(derivative>0) { db_0=db; db.setZero(); }
    if(derivative>1) { ddb_0=ddb; ddb.setZero(); }
    for(uint k=0; k<=K; k++) {
      if(!p) {
        if(!k && t<knotTimes(0)) b(k)=1.;
        else if(k==K && t>=knotTimes(k)) b(k)=1.;
        else if(knotTimes(k)<=t && t<knotTimes(k+1)) b(k)=1.;
      } else {
        if(k+p<knotTimes.N) {
          double xnom = t - knotTimes(k);
          double xden = knotTimes(k+p) - knotTimes(k);
          double x = DIV(xnom, xden, true);
          b(k) = x * b_0(k);
          if(derivative>0) db(k) = DIV(1., xden, true) * b_0(k) + x * db_0(k);
          if(derivative>1) ddb(k) = DIV(2., xden, true) * db_0(k) + x * ddb_0(k);
        }
        if(k<K && k+p+1<knotTimes.N) {
          double ynom = knotTimes(k+p+1) - t;
          double yden = knotTimes(k+p+1) - knotTimes(k+1);
          double y = DIV(ynom, yden, true);
          b(k) += y * b_0(k+1);
          if(derivative>0) db(k) += DIV(-1., yden, true) * b_0(k+1) + y * db_0(k+1);
          if(derivative>1) ddb(k) += DIV(-2., yden, true) * db_0(k+1) + y * ddb_0(k+1);
        }
      }
    }
    if(t<knotTimes(0) || t>=knotTimes.last()) break;
  }
  switch(derivative) {
    case 0:
      return b;
    case 1:
      return db;
    case 2:
      return ddb;
  }
  HALT("Derivate of order " << derivative << " not yet implemented.");
}

#define ZDIV(x,y) (y?x/y:0.)

void Spline::getCoeffs2(arr& b, arr& db, arr& ddb, double t, uint degree, double* knotTimes, uint knotN, uint knotTimesN, uint derivatives) {
  CHECK_EQ(knotN+degree+1, knotTimesN, "");

  b.resize(knotN).setZero();
  if(derivatives>0) db.resize(knotN).setZero();
  if(derivatives>1) ddb.resize(knotN).setZero();

  arr b_prev, db_prev, ddb_prev;
  for(uint p=0; p<=degree; p++) {
    b_prev=b; b.setZero();
    if(derivatives>0) { db_prev=db; db.setZero(); }
    if(derivatives>1) { ddb_prev=ddb; ddb.setZero(); }
    for(uint k=0; k<knotN; k++) {
      if(!p) {
        if(!k && t<knotTimes[0]) b.elem(k)=1.;
        else if(k==knotN-1 && t>=knotTimes[k]) b.elem(k)=1.;
        else if(knotTimes[k]<=t && t<knotTimes[k+1]) b.elem(k)=1.;
      } else {
        if(k+p<knotTimesN) {
          double xnom = t - knotTimes[k];
          double xden = knotTimes[k+p] - knotTimes[k];
          double x = ZDIV(xnom, xden);
          b.elem(k) = x * b_prev.elem(k);
          if(derivatives>0) db.elem(k) = ZDIV(1., xden) * b_prev.elem(k) + x * db_prev.elem(k);
          if(derivatives>1) ddb.elem(k) = ZDIV(2., xden) * db_prev.elem(k) + x * ddb_prev.elem(k);
        }
        if(k<knotN-1 && k+p+1<knotTimesN) {
          double ynom = knotTimes[k+p+1] - t;
          double yden = knotTimes[k+p+1] - knotTimes[k+1];
          double y = ZDIV(ynom, yden);
          b.elem(k) += y * b_prev.elem(k+1);
          if(derivatives>0) db.elem(k) += ZDIV(-1., yden) * b_prev.elem(k+1) + y * db_prev.elem(k+1);
          if(derivatives>1) ddb.elem(k) += ZDIV(-2., yden) * db_prev.elem(k+1) + y * ddb_prev.elem(k+1);
        }
      }
    }
    if(t<knotTimes[0] || t>=knotTimes[knotTimesN-1]) break;
  }
}

void Spline::eval(arr& x, arr& xDot, arr& xDDot, double t) const {
#if 0 //computing coeffs for ALL knot points (most zero...)
//  uint K = knotPoints.d0-1;
//  arr coeffs = getCoeffs(t, K, derivative);
  arr coeffs = getCoeffs2(t, degree, knotTimes.p, knotPoints.d0, knotTimes.N, derivative);
  return (~coeffs * knotPoints).reshape(knotPoints.d1);
#else //pick out only the LOCAL knot points

  //find the first knotTime >t
  int offset = knotTimes.rankInSorted(t, rai::lowerEqual<double>, true);
  offset -= degree+1;
  if(offset<0) offset=0;

  uint knotN = degree+1;
  uint knotTimesN = knotN + 1+degree;
  if(offset+knotTimesN>knotTimes.N) offset = knotTimes.N - knotTimesN;

  //get coeffs
  arr b, db, ddb;
  uint derivative=0;
  if(!!xDot) derivative=1;
  if(!!xDDot) derivative=2;
  getCoeffs2(b, db, ddb, t, degree, knotTimes.p+offset, knotN, knotTimesN, derivative);

  //linear combination
  uint n = knotPoints.d1;
  if(!!x) x.resize(n).setZero();
  if(!!xDot) xDot.resize(n).setZero();
  if(!!xDDot) xDDot.resize(n).setZero();
  for(uint j=0;j<b.N;j++){
    if(!!x) for(uint k=0;k<n;k++) x.elem(k) += b.elem(j)*knotPoints(offset+j,k);
    if(!!xDot) for(uint k=0;k<n;k++) xDot.elem(k) += db.elem(j)*knotPoints(offset+j,k);
    if(!!xDDot) for(uint k=0;k<n;k++) xDDot.elem(k) += ddb.elem(j)*knotPoints(offset+j,k);
  }
#endif
}

arr Spline::eval(double t, uint derivative) const{
  arr x;
  if(derivative==0) eval(x, NoArr, NoArr, t);
  else if(derivative==1) eval(NoArr, x, NoArr, t);
  else if(derivative==2) eval(NoArr, NoArr, x, t);
  else NIY;
  return x;
}

arr Spline::eval(const arr& ts){
  arr f(ts.N, points.d1);
  for(uint i=0;i<ts.N;i++) f[i] = eval(ts(i));
  return f;
}

Spline& Spline::set(uint _degree, const arr& _points, const arr& _times, const arr& startVel, const arr& endVel) {
  CHECK_EQ(_times.nd, 1, "");
  CHECK_LE(_points.nd, 2, "");
  CHECK_EQ(_points.d0, _times.N, "");

  degree = _degree;
  points=_points; if(points.nd==1) points.reshape(points.N, 1);
  times=_times;

  //knot points with head and tail
  knotPoints = points;
  for(uint i=0; i<degree/2; i++) {
    knotPoints.prepend(points[0]);
    knotPoints.append(points[points.d0-1]);
  }

  //knot times with head and tail
  uint m=knotPoints.d0+degree;
  knotTimes.resize(m+1);
  for(uint i=0; i<=m; i++) {
    if(i<=degree) knotTimes(i)=_times.first();
    else if(i>=m-degree) knotTimes(i)=_times.last();
    else if((degree%2)) {
      knotTimes(i) = _times(i-degree);
    } else {
      knotTimes(i) = .5*(_times(i-degree-1)+_times(i-degree));
    }
  }

  //can also tune startVel and endVel for degree 2
  if(!!startVel) setDoubleKnotVel(-1, startVel);
  if(!!endVel) setDoubleKnotVel(points.N-1, endVel);

  CHECK_EQ(knotPoints.d0, knotTimes.N-degree-1 , "");

  return *this;
}

Spline& Spline::set_vel(uint degree, const arr& _points, const arr& velocities, const arr& _times){
  arr pts = repmat(_points,1,2).reshape(-1, _points.d1);
  arr tms = repmat(_times,1,2).reshape(-1);
  set(degree, pts, tms);
  if(velocities.N){
    for(uint t=0;t<velocities.d0;t++){
      setDoubleKnotVel(2*t, velocities[t]);
    }
  }
  return *this;
}

Spline&Spline::setUniform(uint _degree, uint steps) {
  arr x = ::range(0.,1.,steps);
  set(_degree, x, x);
  return *this;
}

arr Spline::getGridBasis(uint T) {
  arr basis(T, knotPoints.d0);
  arr db,ddb;
  for(uint t=0; t<T; t++){
    getCoeffs2(basis[t].noconst(), db, ddb, double(t)/(T-1), degree, knotTimes.p, knotPoints.d0, knotTimes.N, 0);
  }
  return basis;
}

void Spline::append(const arr& _points, const arr& _times){
  CHECK_EQ(_points.nd, 2, "");
  CHECK_EQ(_points.d0, _times.N, "");

  CHECK_GE(_times.first(), 0., "append needs to be in relative time, always with _times.first()>=0.");
  if(!_times.first()){
    CHECK_LE(maxDiff(points[-1], _points[0]), 1e-10, "when appending with _times.first()=0., the first point needs to be identical to the previous last, making this a double knot");
  }

  //remember end time
  double Tend = knotTimes.last();

  points.append(_points);
  times.append(_times+Tend);


  //remove tails
  knotPoints.resize(knotPoints.d0-degree/2, knotPoints.d1);
  knotTimes.resizeCopy(knotTimes.N-1-2*(degree/2));

  //append things:
  knotPoints.append(_points);
  knotTimes.append(_times+Tend);
  if(!(degree%2)){
    for(uint i=knotTimes.N-1;i>=knotTimes.N-_times.N; i--) {
//      times(i) = .5*(times(i-1)+times(i));
      knotTimes(i) = .5*(times(i-degree-1)+times(i-degree));
    }
  }

  //append tails;
  for(uint i=0; i<degree/2; i++) knotPoints.append(_points[-1]);
  knotTimes.append(_times(-1)+Tend,1+2*(degree/2));

  CHECK_EQ(knotPoints.d0, knotTimes.N-degree-1 , "");
}


void Spline::doubleKnot(uint t){
  knotPoints.insRows(t + degree/2);
  knotPoints[t+degree/2] = points[t];

  knotTimes.insert(t+degree+1,times(t));
}

void Spline::setDoubleKnotVel(int t, const arr& vel){
  CHECK_EQ(degree, 2, "NIY");
  arr a=knotPoints[t+degree/2];
  arr b=knotPoints[t+degree/2+1];
  CHECK(maxDiff(a,b)<1e-10,"this is not a double knot!");
  a -= vel*.5*(knotTimes(t+degree+1)-knotTimes(t+degree));
  b += vel*.5*(knotTimes(t+degree+2)-knotTimes(t+degree+1));
}

//==============================================================================

arr Path::getPosition(double t) const {
  return Spline::eval(t);
}

arr Path::getVelocity(double t) const {
  return Spline::eval(t, 1);
}

void Path::transform_CurrentBecomes_EndFixed(const arr& current, double t) {
  arr delta = current - eval(t);
  for(uint i=0; i<knotPoints.d0; i++) {
    double ti = double(i)/double(knotPoints.d0-1);
    double a = (1.-ti)/(1.-t);
    knotPoints[i] += a*delta;
  }
}

void Path::transform_CurrentFixed_EndBecomes(const arr& end, double t) {
  arr delta = end - eval(1.);
  for(uint i=0; i<knotPoints.d0; i++) {
    double ti = double(i)/double(knotPoints.d0-1);
    double a = (ti-t)/(1.-t);
    knotPoints[i] += a*delta;
  }
}

void Path::transform_CurrentBecomes_AllFollow(const arr& current, double t) {
  arr delta = current - eval(t);
  for(uint i=0; i<knotPoints.d0; i++) knotPoints[i] += delta;
}

//==============================================================================

void CubicPiece::set(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau) {
  double tau2 = tau*tau, tau3 = tau*tau2;
  d = x0;
  c = v0;
  b = 1./tau2 * (  3.*(x1-x0) - tau*(v1+2.*v0) );
  a = 1./tau3 * ( -2.*(x1-x0) + tau*(v1+v0) );
}

void CubicPiece::eval(arr& x, arr& xDot, arr& xDDot, double t) const {
  double t2=t*t, t3=t*t2;
  if(!!x){
    x = d;
    x += t*c;
    x += t2*b;
    x += t3*a;
  }
  if(!!xDot){
    xDot = c;
    xDot += (2.*t)*b;
    xDot += (3.*t2)*a;
  }
  if(!!xDDot){
    xDDot = 2.*b;
    xDDot += (6.*t)*a;
  }
}

void CubicSpline::set(const arr& pts, const arr& vels, const arr& _times){
  CHECK_GE(_times.N, 2, "need at least 2 knots");
  times = _times;
  uint K=pts.d0-1;
  pieces.resize(K);
  for(uint k=0;k<K;k++){
    pieces(k).set(pts[k], vels[k], pts[k+1], vels[k+1], times(k+1)-times(k));
  }
}

void CubicSpline::append(const arr& pts, const arr& vels, const arr& _times){
  CHECK_GE(_times(0), 1e-6, "for appending, first time needs to be greater zero");

  //current end state
  arr x, xDot;
  pieces(-1).eval(x, xDot, NoArr, times(-1)-times(-2));

  //new times
  double Tend = times.last();
  times.append(_times+Tend);

  uint K0=pieces.N;
  uint K=pts.d0;
  pieces.resizeCopy(K0+K);
  pieces(K0+0).set(x, xDot, pts[0], vels[0], _times(0));
  for(uint k=1;k<K;k++){
    pieces(K0+k).set(pts[k-1], vels[k-1], pts[k], vels[k], _times(k)-_times(k-1));
  }
}


void CubicSpline::eval(arr& x, arr& xDot, arr& xDDot, double t) const {
  CHECK_GE(times.N, 2, "spline is empty");
  if(t<times(0)){
    pieces(0).eval(x, xDot, xDDot, 0.);
    if(!!xDDot){
      CHECK_ZERO(absMax(xDot), 1e-6, "don't query a cubic spline at neg time for non-zero start velocity");
      xDDot.setZero();
    }
    return;
  }
  if(t>times(-1)){
    pieces(-1).eval(x, xDot, xDDot, times(-1)-times(-2));
    if(!!xDDot){
      CHECK_ZERO(absMax(xDot), 1e-6, "don't query a cubic spline after final time for non-zero final velocity");
      xDDot.setZero();
    }
    return;
  }

  uint k = times.rankInSorted(t,rai::lowerEqual<double>, false);
  if(k<times.N){ CHECK_LE(t, times(k), ""); }
  else{ CHECK_GE(t, times.last(), ""); }
  if(k) k--;
  if(k>=pieces.N-1) k=pieces.N-1;
  //    CHECK_GE(t, times(k), "");
  //    cout <<"t: " <<t <<" k: " <<k <<' ' <<times <<endl;

  pieces(k).eval(x, xDot, xDDot, t-times(k));
}

arr CubicSpline::eval(double t, uint diff) const{
  arr ret;
  if(diff==0) eval(ret, NoArr, NoArr, t);
  else if(diff==1) eval(NoArr, ret, NoArr, t);
  else if(diff==2) eval(NoArr, NoArr, ret, t);
  else NIY;
  return ret;
}

arr CubicSpline::eval(const arr& T, uint diff) const{
  arr x(T.N, pieces.first().d.N);
  for(uint i=0;i<T.N;i++) x[i] = eval(T(i), diff);
  return x;
}

arr CubicSplineLeapCost(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ) {
  arr D = (x1-x0) - (.5*tau)*(v0+v1);
  if(tauJ.N){
    if(!D.jac) D.J().sparse().resize(v0.N, tauJ.d1, 0);
    D.J() -= .5 * (v0+v1).noJ() * tauJ;
  }

  arr V = v1 - v0;

  double s12 = sqrt(12.);

  arr tilD = (s12 * pow(tau, -1.5)) * D;
  if(tauJ.N) tilD.J() += (s12 * (-1.5) * pow(tau,-2.5)) * D.noJ() * tauJ;

  arr tilV = pow(tau, -0.5) * V;
  if(tauJ.N){
    if(!tilV.jac) tilV.J().sparse().resize(v0.N, tauJ.d1, 0);
    tilV.J() += ((-0.5) * pow(tau,-1.5)) * V.noJ() * tauJ;
  }

  arr y;
  y.setBlockVector(tilD, tilV);
  return y;
}

arr CubicSplineMaxAcc(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ){
  double tau2 = tau*tau, tau3 = tau*tau2;
  //  arr d = x0;
  //  arr c = v0;
  arr b = 2./tau2 * (  3.*(x1-x0) - tau*(v1+2.*v0) );
  if(tauJ.N){
    b.J() += -12./tau3 * (x1-x0) * tauJ;
    b.J() -= -2./tau2 * (v1+2.*v0) * tauJ;
  }
  arr a = 6./tau2 * ( -2.*(x1-x0) + tau*(v1+v0) );
  if(tauJ.N){
    a.J() -= -24./tau3 * (x1-x0) * tauJ;
    a.J() += -6./tau2 * (v1+v0) * tauJ;
  }

  uint n=x0.N;
  arr y(4*n);
  if(b.jac) y.J().sparse().resize(y.N, b.jac->d1, 0);
  y.setVectorBlock(b, 0*n);
  y.setVectorBlock(-b, 1*n);
  y.setVectorBlock(b + a, 2*n);
  y.setVectorBlock(-b - a, 3*n);
  return y;
}

arr CubicSplineMaxVel(const arr& x0, const arr& v0, const arr& x1, const arr& v1, double tau, const arr& tauJ){
  double tau2 = tau*tau, tau3 = tau*tau2;
  //  arr d = x0;
  //  arr c = v0;
  arr b = 2./tau2 * (  3.*(x1-x0) - tau*(v1+2.*v0) );
  if(tauJ.N){
    b.J() += -12./tau3 * (x1-x0) * tauJ;
    b.J() -= -2./tau2 * (v1+2.*v0) * tauJ;
  }
  arr a = 6./tau2 * ( -2.*(x1-x0) + tau*(v1+v0) );
  if(tauJ.N){
    a.J() -= -24./tau3 * (x1-x0) * tauJ;
    a.J() += -6./tau2 * (v1+v0) * tauJ;
  }

//  arr t = -b/a;
//  arr vt = b % t + v0;

  uint n=x0.N;
  arr y(4*n);
  y.setZero();
  if(v0.jac) y.J().sparse().resize(y.N, v0.jac->d1, 0);
  y.setVectorBlock(v0, 0*n);
  y.setVectorBlock(-v0, 1*n);
//  y.setVectorBlock(vt, 2*n);
//  y.setVectorBlock(-vt, 3*n);
  return y;
}

} //namespace rai
