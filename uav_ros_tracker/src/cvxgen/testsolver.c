/* Produced by CVXGEN, 2018-11-06 10:21:35 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.x_ss_1[0] = 0.20319161029830202;
  params.x_ss_1[1] = 0.8325912904724193;
  params.x_ss_1[2] = -0.8363810443482227;
  params.x_ss_1[3] = 0.04331042079065206;
  params.Q[0] = 1.8929469543476547;
  params.Q[1] = 1.896293088933438;
  params.Q[2] = 1.1255853104638363;
  params.Q[3] = 1.2072428781381868;
  params.x_ss_2[0] = -1.7941311867966805;
  params.x_ss_2[1] = -0.23676062539745413;
  params.x_ss_2[2] = -1.8804951564857322;
  params.x_ss_2[3] = -0.17266710242115568;
  params.x_ss_3[0] = 0.596576190459043;
  params.x_ss_3[1] = -0.8860508694080989;
  params.x_ss_3[2] = 0.7050196079205251;
  params.x_ss_3[3] = 0.3634512696654033;
  params.x_ss_4[0] = -1.9040724704913385;
  params.x_ss_4[1] = 0.23541635196352795;
  params.x_ss_4[2] = -0.9629902123701384;
  params.x_ss_4[3] = -0.3395952119597214;
  params.x_ss_5[0] = -0.865899672914725;
  params.x_ss_5[1] = 0.7725516732519853;
  params.x_ss_5[2] = -0.23818512931704205;
  params.x_ss_5[3] = -1.372529046100147;
  params.x_ss_6[0] = 0.17859607212737894;
  params.x_ss_6[1] = 1.1212590580454682;
  params.x_ss_6[2] = -0.774545870495281;
  params.x_ss_6[3] = -1.1121684642712744;
  params.x_ss_7[0] = -0.44811496977740495;
  params.x_ss_7[1] = 1.7455345994417217;
  params.x_ss_7[2] = 1.9039816898917352;
  params.x_ss_7[3] = 0.6895347036512547;
  params.x_ss_8[0] = 1.6113364341535923;
  params.x_ss_8[1] = 1.383003485172717;
  params.x_ss_8[2] = -0.48802383468444344;
  params.x_ss_8[3] = -1.631131964513103;
  params.x_ss_9[0] = 0.6136436100941447;
  params.x_ss_9[1] = 0.2313630495538037;
  params.x_ss_9[2] = -0.5537409477496875;
  params.x_ss_9[3] = -1.0997819806406723;
  params.x_ss_10[0] = -0.3739203344950055;
  params.x_ss_10[1] = -0.12423900520332376;
  params.x_ss_10[2] = -0.923057686995755;
  params.x_ss_10[3] = -0.8328289030982696;
  params.x_ss_11[0] = -0.16925440270808823;
  params.x_ss_11[1] = 1.442135651787706;
  params.x_ss_11[2] = 0.34501161787128565;
  params.x_ss_11[3] = -0.8660485502711608;
  params.x_ss_12[0] = -0.8880899735055947;
  params.x_ss_12[1] = -0.1815116979122129;
  params.x_ss_12[2] = -1.17835862158005;
  params.x_ss_12[3] = -1.1944851558277074;
  params.x_ss_13[0] = 0.05614023926976763;
  params.x_ss_13[1] = -1.6510825248767813;
  params.x_ss_13[2] = -0.06565787059365391;
  params.x_ss_13[3] = -0.5512951504486665;
  params.x_ss_14[0] = 0.8307464872626844;
  params.x_ss_14[1] = 0.9869848924080182;
  params.x_ss_14[2] = 0.7643716874230573;
  params.x_ss_14[3] = 0.7567216550196565;
  params.x_ss_15[0] = -0.5055995034042868;
  params.x_ss_15[1] = 0.6725392189410702;
  params.x_ss_15[2] = -0.6406053441727284;
  params.x_ss_15[3] = 0.29117547947550015;
  params.x_ss_16[0] = -0.6967713677405021;
  params.x_ss_16[1] = -0.21941980294587182;
  params.x_ss_16[2] = -1.753884276680243;
  params.x_ss_16[3] = -1.0292983112626475;
  params.x_ss_17[0] = 1.8864104246942706;
  params.x_ss_17[1] = -1.077663182579704;
  params.x_ss_17[2] = 0.7659100437893209;
  params.x_ss_17[3] = 0.6019074328549583;
  params.x_ss_18[0] = 0.8957565577499285;
  params.x_ss_18[1] = -0.09964555746227477;
  params.x_ss_18[2] = 0.38665509840745127;
  params.x_ss_18[3] = -1.7321223042686946;
  params.x_ss_19[0] = -1.7097514487110663;
  params.x_ss_19[1] = -1.2040958948116867;
  params.x_ss_19[2] = -1.3925560119658358;
  params.x_ss_19[3] = -1.5995826216742213;
  params.x_ss_20[0] = -1.4828245415645833;
  params.x_ss_20[1] = 0.21311092723061398;
  params.x_ss_20[2] = -1.248740700304487;
  params.x_ss_20[3] = 1.808404972124833;
  params.x_ss_21[0] = 0.7264471152297065;
  params.x_ss_21[1] = 0.16407869343908477;
  params.x_ss_21[2] = 0.8287224032315907;
  params.x_ss_21[3] = -0.9444533161899464;
  params.x_ss_22[0] = 1.7069027370149112;
  params.x_ss_22[1] = 1.3567722311998827;
  params.x_ss_22[2] = 0.9052779937121489;
  params.x_ss_22[3] = -0.07904017565835986;
  params.x_ss_23[0] = 1.3684127435065871;
  params.x_ss_23[1] = 0.979009293697437;
  params.x_ss_23[2] = 0.6413036255984501;
  params.x_ss_23[3] = 1.6559010680237511;
  params.x_ss_24[0] = 0.5346622551502991;
  params.x_ss_24[1] = -0.5362376605895625;
  params.x_ss_24[2] = 0.2113782926017822;
  params.x_ss_24[3] = -1.2144776931994525;
  params.x_ss_25[0] = -1.2317108144255875;
  params.x_ss_25[1] = 0.9026784957312834;
  params.x_ss_25[2] = 1.1397468137245244;
  params.x_ss_25[3] = 1.8883934547350631;
  params.x_ss_26[0] = 1.4038856681660068;
  params.x_ss_26[1] = 0.17437730638329096;
  params.x_ss_26[2] = -1.6408365219077408;
  params.x_ss_26[3] = -0.04450702153554875;
  params.x_ss_27[0] = 1.7117453902485025;
  params.x_ss_27[1] = 1.1504727980139053;
  params.x_ss_27[2] = -0.05962309578364744;
  params.x_ss_27[3] = -0.1788825540764547;
  params.x_ss_28[0] = -1.1280569263625857;
  params.x_ss_28[1] = -1.2911464767927057;
  params.x_ss_28[2] = -1.7055053231225696;
  params.x_ss_28[3] = 1.56957275034837;
  params.x_ss_29[0] = 0.5607064675962357;
  params.x_ss_29[1] = -1.4266707301147146;
  params.x_ss_29[2] = -0.3434923211351708;
  params.x_ss_29[3] = -1.8035643024085055;
  params.x_ss_30[0] = -1.1625066019105454;
  params.x_ss_30[1] = 0.9228324965161532;
  params.x_ss_30[2] = 0.6044910817663975;
  params.x_ss_30[3] = -0.0840868104920891;
  params.x_ss_31[0] = -0.900877978017443;
  params.x_ss_31[1] = 0.608892500264739;
  params.x_ss_31[2] = 1.8257980452695217;
  params.x_ss_31[3] = -0.25791777529922877;
  params.x_ss_32[0] = -1.7194699796493191;
  params.x_ss_32[1] = -1.7690740487081298;
  params.x_ss_32[2] = -1.6685159248097703;
  params.x_ss_32[3] = 1.8388287490128845;
  params.x_ss_33[0] = 0.16304334474597537;
  params.x_ss_33[1] = 1.3498497306788897;
  params.x_ss_33[2] = -1.3198658230514613;
  params.x_ss_33[3] = -0.9586197090843394;
  params.x_ss_34[0] = 0.7679100474913709;
  params.x_ss_34[1] = 1.5822813125679343;
  params.x_ss_34[2] = -0.6372460621593619;
  params.x_ss_34[3] = -1.741307208038867;
  params.x_ss_35[0] = 1.456478677642575;
  params.x_ss_35[1] = -0.8365102166820959;
  params.x_ss_35[2] = 0.9643296255982503;
  params.x_ss_35[3] = -1.367865381194024;
  params.x_ss_36[0] = 0.7798537405635035;
  params.x_ss_36[1] = 1.3656784761245926;
  params.x_ss_36[2] = 0.9086083149868371;
  params.x_ss_36[3] = -0.5635699005460344;
  params.x_ss_37[0] = 0.9067590059607915;
  params.x_ss_37[1] = -1.4421315032701587;
  params.x_ss_37[2] = -0.7447235390671119;
  params.x_ss_37[3] = -0.32166897326822186;
  params.x_ss_38[0] = 1.5088481557772684;
  params.x_ss_38[1] = -1.385039165715428;
  params.x_ss_38[2] = 1.5204991609972622;
  params.x_ss_38[3] = 1.1958572768832156;
  params.x_ss_39[0] = 1.8864971883119228;
  params.x_ss_39[1] = -0.5291880667861584;
  params.x_ss_39[2] = -1.1802409243688836;
  params.x_ss_39[3] = -1.037718718661604;
  params.x_ss_40[0] = 1.3114512056856835;
  params.x_ss_40[1] = 1.8609125943756615;
  params.x_ss_40[2] = 0.7952399935216938;
  params.x_ss_40[3] = -0.07001183290468038;
  params.Af[0] = -0.8518009412754686;
  params.Af[1] = 1.3347515373726386;
  params.Af[2] = 1.4887180335977037;
  params.Af[3] = -1.6314736327976336;
  params.Af[4] = -1.1362021159208933;
  params.Af[5] = 1.327044361831466;
  params.Af[6] = 1.3932155883179842;
  params.Af[7] = -0.7413880049440107;
  params.Af[8] = -0.8828216126125747;
  params.x_0[0] = -0.27673991192616;
  params.x_0[1] = 0.15778600105866714;
  params.x_0[2] = -1.6177327399735457;
  params.x_0[3] = 1.3476485548544606;
  params.Bf[0] = 0.13893948140528378;
  params.A[0] = 1.0998712601636944;
  params.A[1] = -1.0766549376946926;
  params.A[2] = 1.8611734044254629;
  params.A[3] = 1.0041092292735172;
  params.A[4] = -0.6276245424321543;
  params.A[5] = 1.794110587839819;
  params.A[6] = 0.8020471158650913;
  params.A[7] = 1.362244341944948;
  params.A[8] = -1.8180107765765245;
  params.B[0] = -1.7774338357932473;
  params.x_max_2[0] = 0.9709490941985153;
  params.x_min_2[0] = -0.7812542682064318;
  params.x_max_3[0] = 0.0671374633729811;
  params.x_min_3[0] = -1.374950305314906;
  params.x_max_4[0] = 1.9118096386279388;
  params.x_min_4[0] = 0.011004190697677885;
  params.u_max[0] = 1.3160043138989015;
  params.u_min[0] = -1.7038488148800144;
}
