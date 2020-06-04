#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {
      t_ = t;
      r_ = r;
  }

  explicit RigTForm(const Cvec3& t) {
      t_ = t;
      r_ = Quat();
  }

  explicit RigTForm(const Quat& r) {
      t_ = Cvec3 (0,0,0);
      r_ = r;
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
      Cvec3 temp;
      if (a[3] == 1)
          temp = (this->r_) * (Cvec3 (a[0], a[1], a[2])) + (this->t_) ;
      else
          temp = (this->r_) * (Cvec3 (a[0], a[1], a[2]));
      return Cvec4 (temp[0], temp[1], temp[2], a[3]);
  }

  RigTForm operator * (const RigTForm& a) const {
      Cvec3 res_t = (this->t_) + (this->r_) * (a.t_);
      Quat res_q = (this->r_) * a.r_;
      return RigTForm(res_t, res_q);
  }
};

inline RigTForm inv(const RigTForm& tform) {
    Cvec3 res_t = -(inv(tform.getRotation()) * tform.getTranslation());
    Quat res_q = inv(tform.getRotation());
    return RigTForm(res_t, res_q);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
    Matrix4 r = quatToMatrix(tform.getRotation());
    Cvec3 a = tform.getTranslation();
    r(0,3) = a[0];
    r(1,3) = a[1];
    r(2,3) = a[2];
//    Matrix4 t = Matrix4::makeTranslation(tform.getTranslation());
//    Matrix4 m = t * r;
  return r;
}

#endif
