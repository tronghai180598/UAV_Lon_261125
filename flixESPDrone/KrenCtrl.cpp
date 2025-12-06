#include "KrenCtrl.hpp"
#include <string>
#include <math.h>

#define _slCtrl_2PD 1
#define _slCtrl_Modal 2
#define _slCtrl_Slide 3
#define _slCtrl_MaxFs 4
#define __SelCtrl 3

KrenCtrl::KrenCtrl(float Tf, float Tv, float Tm, float Tmi){
  mTi = 0.2;
  mTe = 0.04; // инициализация времени интегратора
  mTmu = mTm;
  setParam(Tf, Tv, Tm);
  setCtrlParam();
}

KrenCtrl::KrenCtrl(){
  //mTmm = mTm; mTm = 0; 
  klmf = klmv = 0.1;
  mTi = 0.25;
  mTe = 0.1;
  mTmu = 0.05;
  mTm = 0.05;
  setCtrlParam();
}
void KrenCtrl::reset()
{
    Us = 0.0f;
    Uv = 0.0f;
    Ui = 0.0f;
    Um = 0.0f;
    Umm = 0.0f;
    erVi1 = 0.0f;
    mdVi  = 0.0f;
    muMd  = 0.0f;
    uMold = 0.0f;
    oUi   = 0.0f;
}

void KrenCtrl::setCtrlParam(){
#if (__SelCtrl==_slCtrl_2PD)
  //mTmu = mTm; //sqrt(mTm*mTm + mTmm*mTmm);
  // mTi = 0.25;
  // mTe = 0.1;
  // mTmu = 0.05;
  // mTm = 0.05;
  Kpv = mTe*mTv/(25.0*mTmu*mTmu);
  Kdv = mTe*mTv/(2.0*mTmu); // SO 4--5
  Kpf = mTf / (15 * mTmu); // MO  15-30
/* -Fi       -Vi        Us
    |   ___   |   ___   |      ___    _______       _______ Ui Um 
Fs--O--|Kpf|--O--|KpV|--O--O--|sat|--|1/(sTe)|--*--|1/(sTi)|--O-->u
       |___|Vs | |___|  |  |  |___|  |_______|  |Uv|_______|  |  
              _|_____   |  \--------------------*-------------/1  
             |s Kdv  |  |           
             |1+s Tmu|--/
             |_______|
*/
#endif
#if (__SelCtrl==_slCtrl_Modal)
  Kd  = mTmi / ( mTm * 2 );
  Kp  = mTv / ( 2.23 * 2 * mTm);
  Kpf = mTf / ( 2.23 * 4 * mTm );
  Kdf = 1.25 * mTm * Kpf / mTf;
#endif
#if (__SelCtrl==_slCtrl_Slide)
  // mTi = mTe;
  // mTmu = mTm;
  //Kd  = 1000;
  Kdv = 0.05;
  Kpv  = 0.1;
  Kpf = 1.0;
#endif
#if (__SelCtrl==_slCtrl_MaxFs)
/*   __       ___   __            __   __   ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi__Kp__|s |_|Kd|_|_1__|_uM_
 Fi_|- |       Vi__|- |  |AC|*Ac_|g |      |sTmi|
    |__|           |__|          |n_|      |____|
*/
  Kd  = mTmi / ( mTm * 2 );
  Kp  = mTv / ( 2 * mTm);
  Kpm  = 2*Kd/(mTv*mTmi);
  Kpf = mTf / ( 2.2 * 2.5 * mTm ); // opt
  //Kpf = mTf / ( 5 * 5 * mTm );
#endif
}

//#define stabkalm 1
//#define stabkalm 0.5
#define stabkalm 0.05
//#define stabkalm 0.01
//#define __Kalman
#ifdef  __Kalman
  #define __KlFmn 0.0042
  #define __KlFmx 0.52
  #define __KlVmn 0.042
  #define __KlVmx 0.52
#else
  #define __KlFmn stabkalm
  #define __KlFmx stabkalm
  #define __KlVmn stabkalm
  #define __KlVmx stabkalm
#endif // 

float lineTrans(float x, float x1, float x2, float y1, float y2){
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}
void KrenCtrl::UdateKalman(float Fi, float Vi){
  //mFi = Fi; mVi = Vi; return;
  float uI2 = ((Ui/100)*(Ui/100));
  float kKFi = lineTrans(uI2, 0 , 100 , __KlFmn, __KlFmx); //kKFi = 1.0;
  float kKVi = lineTrans(uI2, 0 , 100 , __KlVmn, __KlVmx); //kKVi = 1.0;
  mFi += (Fi - mFi) * klmf;
  mVi += (Vi - mVi) * klmv;
}

float KrenCtrl::updateCtrl(float dt, float setFi, float Fi, float Vi){
  UdateKalman(Fi, Vi);  
  float erFi = setFi - mFi;
  float erVi = Kpf * erFi  - mVi;
  rldiff(dt, erVi - erVi1, 1.0, mTmu, mdVi); erVi1 = erVi;
#if (__SelCtrl==_slCtrl_2PD)
/* -Fi       -Vi        Us
    |   ___Vs |   ___   |      ___    _______       _______ Ui Um
Fs--O--|Kpf|--O--|KpV|--O--O--|sat|--|1/(sTe)|--*--|1/(sTi)|--O-->u
       |___|   | |___|  |  |  |___|  |_______|  |Uv|_______|  |  
              _|_____   |  \--------------------*-------------/  
             |s Kdv  |  |           
             |1+s Tmu|--/
             |_______|mdVi
*/
  Us =  erVi * Kpv + mdVi * Kdv;
  Us -= Uv;
#endif
#if (__SelCtrl==_slCtrl_Modal)
/*   __       ___   __       __   __         ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi_|Kv|_|+ |_|Kdv|_|_1__|_uM_
 Fi_|- |       Vi__|- |       Ac_|- |       |sTmi|
    |__|           |__|          |__|       |____|
*/
  //               {---eVi-|---eFi-----|------}
  uI = Kd * ( Kp * ( Kpf * (setFi - mFi) - mVi) - getTr() );
#endif //(__SelCtrl==_slCtrl_Modal)
#if (__SelCtrl==_slCtrl_Slide)
/*   __       ___   __       __   __   ___   ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi_|Kp|_|s |_|Kdv|_|_1__|_uM_
 Fi_|- |       Vi__|- |       Ac_|g |       |sTmi|
    |__|           |__|          |n_|       |____|
*/

 // float eVi = Kpf * (setFi - mFi) - mVi;
  //eVi = 1000 - mVi;

  float s = (Kpv * erVi + Kdv * mdVi) - Uv;

  // tham số chỉnh tay
  const float s_dead = 0.001f;   // deadzone quanh 0
  const float phi    = 0.02f;    // "độ dày lớp biên", thử 0.01–0.05
  const float Umax   = 1000.0f;  // biên điều khiển như trước

  // ----- Luật điều khiển SMC với boundary layer -----
  if (fabsf(s) < s_dead) {
      // rất gần mặt trượt -> tắt điều khiển, tránh rung
      Us = 0.0f;
  } else {
      // dùng sat(s/phi) thay vì sign(s)
      float x = s / phi;
      if (x >  1.0f) x =  1.0f;
      if (x < -1.0f) x = -1.0f;

      Us = Umax * x;
  }

  if (Us * oUi < 0.0f) {
      Us = 0.0f;
  }
  oUi = Us;

#endif //(__SelCtrl==_slCtrl_Slide)
#if (__SelCtrl==_slCtrl_MaxFs)
/*   __       ___   __            __   ___   ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi__Kp__|s |_|Kdv|_|_1__|_uM_
 Fi_|- |       Vi__|- |  |AC|*Ac_|g |       |sTmi|
    |__|           |__|          |n_|       |____|
*/
  float eVi = Kpf * (setFi - mFi) - mVi;

  float xp = getTr();
  if( (eVi*eVi + 1.4*xp*xp) < 2000000 ){
    uI = Kd * ( Kp * eVi - getTr() );
  }
  else{
    // maxFast method
    if(xp > 0) xp = getTr(); else xp = -getTr();
    uI = ( ( Kpm * eVi  + xp ) > 0 )? 1000:(-1000);
    if((uI * oUi) < 0) {  oUi = uI;  uI = 0; } oUi = uI; // через ноль
 }
#endif //(__SelCtrl==_slCtrl_MaxFs)
 
  saturate(Us, -1000.0,1000.0);
  integr(dt, Us, mTe, Uv); // Uv/Us = 1/(1 + s mTe)
  saturate(Uv, -1000.0,1000.0);
  integr(dt, Uv, mTi, Ui);
  saturate(Ui, -1000.0,1000.0);
  Um = Ui + Uv; // Um/Uv = 1 + 1/(s mTi) = (1 + s mTi)/( s mTi)
  saturate(Um, -1000.0,1000.0); // Um/Us = (1 + s mTi) / ( (1 + s mTe) s mTi); if mTi = mTe then Um/Us = 1 / (s mTe)

  intert(dt, Um,1.0,mTmu,Umm);
  rldiff(dt, Umm-uMold, mTmu, mTmu, muMd); uMold = Umm;
  updateMdl(dt, muMd);
  //updateMdl(dt, Uv);
  return Um;
}

float KrenCtrl::GetUi() { return Um; }
