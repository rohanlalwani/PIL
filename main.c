/*gci code by ee.rohan*/

#include "F28x_Project.h"
#include "math.h"
float line_freq = 50, theta = 0;
float Vm = 325, va = 0, vb = 0, vc = 0, vd = 0, vq = 0, f, sampling_time = 100e-6;
float ia = 0, ib = 0, ic = 0, id = 0, iq = 0, pdq = 0, qdq = 0;
float delta_f = 0, delta_f_pre = 0, vq_pre = 0, thetaPLL = 0, thetaPLL1 = 0, thetaPLL_pre = 0, f_pre = 0, kp_pll = 0.687, ki_pll = 77.47;
float L = 50e-3, rL = 0.1;
float b0, b1, b2, b3, b4, b5, b6, b7, b8, b9;
float refa = 0; refb = 0, refc = 0;
float vdc = 800, vao = 0, vbo = 0, vco = 0, vno = 0;
float vla = 0, vlb = 0, vlc = 0, vla_pre = 0, vlb_pre = 0, vlc_pre = 0;
float pref = 3000, qref = 0, omegaL;
float error_p = 0, error_q = 0, error_id = 0, error_iq = 0, idref = 0, iqref = 0, ud = 0, uq = 0;
float error_p_pre = 0, error_q_pre = 0, error_id_pre = 0, error_iq_pre = 0;
float kp_p = 2e-3, ki_p = 1, kp_q = 2e-3, ki_q = 1, kp_i = 200, ki_i = 50;
float md, mq, ma, mb, mc;
Uint16 dac1 = 0, dac2 = 0;

extern interrupt void epwm1_isr(void);
void Gpio_select(void);
void epwm1_int_setup(void);
void setup_epwm(void);
void initdaca(void);
void inverter (void);
void main(void){
    InitSysCtrl();
    DINT;
    Gpio_select();
    setup_epwm();
    epwm1_int_setup();
    initdaca();
    b0 = (2*kp_pll + ki_pll*sampling_time)/2;
    b1 = (ki_pll*sampling_time - 2*kp_pll)/2;
    b2 = (2 * L + rL * sampling_time)/sampling_time;
    b3 = (rL * sampling_time - 2 * L)/sampling_time;
    b4 = (2 * kp_p + ki_p * sampling_time)/2;
    b5 = (ki_p * sampling_time - 2 * kp_p)/2;
    b6 = (2 * kp_q + ki_q * sampling_time)/2;
    b7 = (ki_q * sampling_time - 2 * kp_q)/2;
    b8 = (2 * kp_i + ki_i * sampling_time)/2;
    b9 = (ki_i * sampling_time - 2 * kp_i)/2;
    omegaL = 2 * M_PI * line_freq * L;
    while(1){

    }
}

void Gpio_select(void){
    EALLOW;
    //Enable pin gpio 1 to 5 as epwm 1,2,3
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
    EDIS;
}

void setup_epwm(void){

    EPwm1Regs.TBCTL.bit.CLKDIV = 1;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;
    EPwm1Regs.TBPRD = 2500;
    EPwm1Regs.CMPA.bit.CMPA = 1250;
    EPwm1Regs.AQCTLA.all = 0x0012;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;
    EPwm1Regs.ETSEL.bit.INTSELCMP = 0;
    EPwm1Regs.ETSEL.bit.INTEN = 1;
    EPwm1Regs.ETSEL.bit.INTSEL = 2;
    EPwm1Regs.ETPS.bit.INTPRD = 1;
    EPwm1Regs.ETCLR.bit.INT = 1;

    EPwm2Regs.TBCTL.bit.CLKDIV = 1;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 1250;
    EPwm2Regs.AQCTLA.all = 0x0012;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = 0;

    EPwm3Regs.TBCTL.bit.CLKDIV = 1;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1;
    EPwm3Regs.TBCTL.bit.CTRMODE = 0;
    EPwm3Regs.TBPRD = 2500;
    EPwm3Regs.CMPA.bit.CMPA = 1250;
    EPwm3Regs.AQCTLA.all = 0x0012;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = 0;

}

void epwm1_int_setup(void){
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EALLOW;
    PieVectTable.EPWM1_INT = &epwm1_isr;
    EDIS;
    IER |= M_INT3;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    EINT;
    ERTM;
}

void initdaca(void){
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1;
    DacaRegs.DACCTL.bit.LOADMODE = 0;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
    DacaRegs.DACVALS.bit.DACVALS = dac1;
    DELAY_US(10);
    DacbRegs.DACCTL.bit.DACREFSEL = 1;
    DacbRegs.DACCTL.bit.LOADMODE = 0;
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
    DacbRegs.DACVALS.bit.DACVALS = dac2;
    DELAY_US(10);
    EDIS;
}

extern interrupt void epwm1_isr(void){
    theta = theta + (2*M_PI*line_freq*sampling_time);
    va = Vm*__sin(theta);
    vb = Vm*__sin(theta - 2*M_PI/3);
    vc = Vm*__sin(theta - 4*M_PI/3);
    inverter();
    vd = (2.0/3.0)*((va*__sin(thetaPLL))+(vb*__sin(thetaPLL - 2*M_PI/3))+(vc*__sin(thetaPLL - 4*M_PI/3)));
    vq = (2.0/3.0)*((va*__cos(thetaPLL))+(vb*__cos(thetaPLL - 2*M_PI/3))+(vc*__cos(thetaPLL - 4*M_PI/3)));
    delta_f = delta_f_pre + b0*vq + b1*vq_pre;
    vq_pre = vq;
    delta_f_pre = delta_f;
    f = delta_f + line_freq;
    thetaPLL1 = thetaPLL_pre + M_PI*sampling_time*(f + f_pre);
    thetaPLL = (thetaPLL1 <= 2*M_PI)*thetaPLL1;
    thetaPLL_pre = thetaPLL;
    f_pre = f;
//    DacaRegs.DACVALS.bit.DACVALS = (0.5 + (va/Vm)/2.0)*4095;
//    DacbRegs.DACVALS.bit.DACVALS = (thetaPLL/(2*M_PI))*4095;
    id = (2.0/3.0)*((ia*__sin(thetaPLL))+(ib*__sin(thetaPLL - 2*M_PI/3))+(ic*__sin(thetaPLL - 4*M_PI/3)));
    iq = (2.0/3.0)*((ia*__cos(thetaPLL))+(ib*__cos(thetaPLL - 2*M_PI/3))+(ic*__cos(thetaPLL - 4*M_PI/3)));
    pdq = 1.5 * vd * id;
    qdq = 1.5 * vd * iq;

    error_p = pref - pdq;
    idref = idref + (b4 * error_p) + (b5 * error_p_pre);
    error_p_pre = error_p;

    error_q = qref - qdq;
    iqref = iqref + (b6 * error_q) + (b7 * error_q_pre);
    error_q_pre = error_q;

    error_id = idref - id;
    ud = ud + (b8 * error_id) + (b9 * error_id_pre);
    error_id_pre = error_id;

    error_iq = iqref - iq;
    uq = uq + (b8 * error_iq) + (b9 * error_iq_pre);
    error_iq_pre = error_iq;

    md = (((ud - iq * omegaL) + vd)*2)/vdc;
    mq = (((uq + id * omegaL) + vq)*2)/vdc;

    ma = __sin(thetaPLL) * md + __cos(thetaPLL) * mq;
    mb = __sin(thetaPLL - 2*M_PI/3) * md + __cos(thetaPLL - 2*M_PI/3) * mq;
    mc = __sin(thetaPLL - 4*M_PI/3) * md + __cos(thetaPLL - 4*M_PI/3) * mq;

    refa = (ma + 1)/2;
    refb = (mb + 1)/2;
    refc = (mc + 1)/2;

    EPwm1Regs.CMPA.bit.CMPA = refa * 2500;
    EPwm2Regs.CMPA.bit.CMPA = refb * 2500;
    EPwm3Regs.CMPA.bit.CMPA = refc * 2500;

    dac1 = ((va + 325)/(2 * 325))*4095;
    dac2 = ((ia + 10)/(2 * 10))*4095;
    DacaRegs.DACVALS.bit.DACVALS = dac1;
    DacbRegs.DACVALS.bit.DACVALS = dac2;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    EPwm1Regs.ETCLR.bit.INT = 1;
}

void inverter (void){
    vao = vdc * refa;
    vbo = vdc * refb;
    vco = vdc * refc;
    vno = (vao + vbo + vco)/3;
    vla = vao - va - vno;
    vlb = vbo - vb - vno;
    vlc = vco - vc - vno;
    ia = (-b3 * ia + (vla + vla_pre))/b2;
    ib = (-b3 * ib + (vlb + vlb_pre))/b2;
    ic = (-b3 * ic + (vlc + vlc_pre))/b2;
    vla_pre = vla;
    vlb_pre = vlb;
    vlc_pre = vlc;
}
