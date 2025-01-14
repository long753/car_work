#include "FuzzyControl.h"
//输入增益 如下 P*2 D*5 输出增益为4
float Fuzzy::Fuzzy_Kp(float P, float D) {
  // P = P * 2;
  // D = D * 5;
  /*输入量P语言值特征点*/
  float PFF[7] = {-150, -80, -30, 0, 30, 80, 150};
                //-60, -45, -30, 0, 30, 45, 60
  /*输入量D语言值特征点*/
  float DFF[7] = {-12, -8, -4, 0, 4, 8, 12};
                //-12, -8, -4, 0, 4, 8, 12
  /*输出量U语言值特征点(根据赛道类型选择不同的输出值)*/
  //float UFF[7] = {0.18, 0.2, 0.25, 0.3, 0.4, 0.5, 0.58};
  float UFF[7] = {0.18, 0.2, 0.25, 0.3, 0.35, 0.4, 0.48};
                //0.25, 0.3, 0.35, 0.5, 0.56, 0.58, 0.6
  int rule[7][7] = {
    {6, 5, 4, 4, 2, 1, 1}, //   -3
    {5, 4, 3, 3, 1, 1, 1}, //   -2
    {4, 3, 2, 2, 0, 1, 1}, //   -1
    {2, 1, 1, 1, 1, 1, 2}, //    0
    {1, 1, 0, 2, 2, 3, 4}, //    1
    {1, 1, 1, 3, 3, 4, 5}, //    2
    {1, 1, 2, 4, 4, 5, 6}

  }; 
  float U = 0; /*偏差,偏差微分以及输出值的精确量*/
  float PF[2] = {0}, DF[2] = {0}, UF[4] = {0};
  /*偏差,偏差微分以及输出值的隶属度*/
  int Pn = 0, Dn = 0, Un[4] = {0};
  float t1 = 0, t2 = 0, t3 = 0, t4 = 0, temp1 = 0, temp2 = 0;
  /*隶属度的确定*/
  /*根据PD的指定语言值获得有效隶属度*/
  if (P > PFF[0] && P < PFF[6]) {
    if (P <= PFF[1]) {
      Pn = -2;
      PF[0] = (PFF[1] - P) / (PFF[1] - PFF[0]);
    } else if (P <= PFF[2]) {
      Pn = -1;
      PF[0] = (PFF[2] - P) / (PFF[2] - PFF[1]);
    } else if (P <= PFF[3]) {
      Pn = 0;
      PF[0] = (PFF[3] - P) / (PFF[3] - PFF[2]);
    } else if (P <= PFF[4]) {
      Pn = 1;
      PF[0] = (PFF[4] - P) / (PFF[4] - PFF[3]);
    } else if (P <= PFF[5]) {
      Pn = 2;
      PF[0] = (PFF[5] - P) / (PFF[5] - PFF[4]);
    } else if (P <= PFF[6]) {
      Pn = 3;
      PF[0] = (PFF[6] - P) / (PFF[6] - PFF[5]);
    }
  } else if (P <= PFF[0]) {
    Pn = -2;
    PF[0] = 1;
  } else if (P >= PFF[6]) {
    Pn = 3;
    PF[0] = 0;
  }
  PF[1] = 1 - PF[0];
  // 判断D的隶属度
  if (D > DFF[0] && D < DFF[6]) {
    if (D <= DFF[1]) {
      Dn = -2;
      DF[0] = (DFF[1] - D) / (DFF[1] - DFF[0]);
    } else if (D <= DFF[2]) {
      Dn = -1;
      DF[0] = (DFF[2] - D) / (DFF[2] - DFF[1]);
    } else if (D <= DFF[3]) {
      Dn = 0;
      DF[0] = (DFF[3] - D) / (DFF[3] - DFF[2]);
    } else if (D <= DFF[4]) {
      Dn = 1;
      DF[0] = (DFF[4] - D) / (DFF[4] - DFF[3]);
    } else if (D <= DFF[5]) {
      Dn = 2;
      DF[0] = (DFF[5] - D) / (DFF[5] - DFF[4]);
    } else if (D <= DFF[6]) {
      Dn = 3;
      DF[0] = (DFF[6] - D) / (DFF[6] - DFF[5]);
    }
  }
  // 不在给定的区间内
  else if (D <= DFF[0]) {
    Dn = -2;
    DF[0] = 1;
  } else if (D >= DFF[6]) {
    Dn = 3;
    DF[0] = 0;
  }
  DF[1] = 1 - DF[0];
  /*使用误差范围优化后的规则表rule[7][7]*/
  /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
  /*一般都是四个规则有效*/
  Un[0] = rule[Pn - 1 + 3][Dn - 1 + 3];
  Un[1] = rule[Pn + 3][Dn - 1 + 3];
  Un[2] = rule[Pn - 1 + 3][Dn + 3];
  Un[3] = rule[Pn + 3][Dn + 3];
  if (PF[0] <= DF[0]) // 求小
    UF[0] = PF[0];
  else
    UF[0] = DF[0];
  if (PF[1] <= DF[0])
    UF[1] = PF[1];
  else
    UF[1] = DF[0];
  if (PF[0] <= DF[1])
    UF[2] = PF[0];
  else
    UF[2] = DF[1];
  if (PF[1] <= DF[1])
    UF[3] = PF[1];
  else
    UF[3] = DF[1];
  /*同隶属函数输出语言值求大*/
  if (Un[0] == Un[1]) {
    if (UF[0] > UF[1])
      UF[1] = 0;
    else
      UF[0] = 0;
  }
  if (Un[0] == Un[2]) {
    if (UF[0] > UF[2])
      UF[2] = 0;
    else
      UF[0] = 0;
  }
  if (Un[0] == Un[3]) {
    if (UF[0] > UF[3])
      UF[3] = 0;
    else
      UF[0] = 0;
  }
  if (Un[1] == Un[2]) {
    if (UF[1] > UF[2])
      UF[2] = 0;
    else
      UF[1] = 0;
  }
  if (Un[1] == Un[3]) {
    if (UF[1] > UF[3])
      UF[3] = 0;
    else
      UF[1] = 0;
  }
  if (Un[2] == Un[3]) {
    if (UF[2] > UF[3])
      UF[3] = 0;
    else
      UF[2] = 0;
  }
  t1 = UF[0] * UFF[Un[0]];
  t2 = UF[1] * UFF[Un[1]];
  t3 = UF[2] * UFF[Un[2]];
  t4 = UF[3] * UFF[Un[3]];
  temp1 = t1 + t2 + t3 + t4;
  temp2 = UF[0] + UF[1] + UF[2] + UF[3]; // 模糊量输出
  U = temp1 / temp2;

  U = U * 4; // 4
  return U;
}
// 输入增益为1.5倍输出增益为1倍
float Fuzzy::Fuzzy_Kd(float P, float D) {
  //P = P * 1.5;

  /*输入量P语言值特征点*/
  float PFF[7] = {-150, -100, -20, 0, 20, 100, 150};
  /*输入量D语言值特征点*/
  float DFF[7] = {-30, -15, -8, 0, 8, 15, 30};
  /*输出量U语言值特征点(根据赛道类型选择不同的输出值)*/
  //float UFF[7] = {3.78, 3.83, 3.86, 3.89, 3.92,3.95, 4.0}; 
  float UFF[7] = {2.5, 3.0, 3.5, 3.8, 3.92,3.95, 4.0};// 建议在模糊表中 体现直弯道区别  减少更改 UFF
  int rule[7][7] = {
      {6, 5, 4, 3, 5, 6, 6}, 
      {5, 4, 3, 2, 5, 5, 6}, //   -2
      {5, 4, 3, 1, 5, 5, 5},                        //   -1
      {4, 3, 1, 0, 1, 3, 4},                        //    0
      {5, 5, 5, 1, 3, 4, 5},                        //    1
      {6, 5, 5, 2, 3, 4, 5},                        //    2
      {6, 6, 5, 3, 4, 5, 6}                         //    3
  };
  float U = 0; /*偏差,偏差微分以及输出值的精确量*/
  float PF[2] = {0}, DF[2] = {0}, UF[4] = {0};
  /*偏差,偏差微分以及输出值的隶属度*/
  int Pn = 0, Dn = 0, Un[4] = {0};
  float t1 = 0, t2 = 0, t3 = 0, t4 = 0, temp1 = 0, temp2 = 0;
  /*隶属度的确定*/
  /*根据PD的指定语言值获得有效隶属度*/
  if (P > PFF[0] && P < PFF[6]) {
    if (P <= PFF[1]) {
      Pn = -2;
      PF[0] = (PFF[1] - P) / (PFF[1] - PFF[0]);
    } else if (P <= PFF[2]) {
      Pn = -1;
      PF[0] = (PFF[2] - P) / (PFF[2] - PFF[1]);
    } else if (P <= PFF[3]) {
      Pn = 0;
      PF[0] = (PFF[3] - P) / (PFF[3] - PFF[2]);
    } else if (P <= PFF[4]) {
      Pn = 1;
      PF[0] = (PFF[4] - P) / (PFF[4] - PFF[3]);
    } else if (P <= PFF[5]) {
      Pn = 2;
      PF[0] = (PFF[5] - P) / (PFF[5] - PFF[4]);
    } else if (P <= PFF[6]) {
      Pn = 3;
      PF[0] = (PFF[6] - P) / (PFF[6] - PFF[5]);
    }
  } else if (P <= PFF[0]) {
    Pn = -2;
    PF[0] = 1;
  } else if (P >= PFF[6]) {
    Pn = 3;
    PF[0] = 0;
  }
  PF[1] = 1 - PF[0];
  // 判断D的隶属度
  if (D > DFF[0] && D < DFF[6]) {
    if (D <= DFF[1]) {
      Dn = -2;
      DF[0] = (DFF[1] - D) / (DFF[1] - DFF[0]);
    } else if (D <= DFF[2]) {
      Dn = -1;
      DF[0] = (DFF[2] - D) / (DFF[2] - DFF[1]);
    } else if (D <= DFF[3]) {
      Dn = 0;
      DF[0] = (DFF[3] - D) / (DFF[3] - DFF[2]);
    } else if (D <= DFF[4]) {
      Dn = 1;
      DF[0] = (DFF[4] - D) / (DFF[4] - DFF[3]);
    } else if (D <= DFF[5]) {
      Dn = 2;
      DF[0] = (DFF[5] - D) / (DFF[5] - DFF[4]);
    } else if (D <= DFF[6]) {
      Dn = 3;
      DF[0] = (DFF[6] - D) / (DFF[6] - DFF[5]);
    }
  }
  // 不在给定的区间内
  else if (D <= DFF[0]) {
    Dn = -2;
    DF[0] = 1;
  } else if (D >= DFF[6]) {
    Dn = 3;
    DF[0] = 0;
  }
  DF[1] = 1 - DF[0];
  /*使用误差范围优化后的规则表rule[7][7]*/
  /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
  /*一般都是四个规则有效*/
  Un[0] = rule[Pn - 1 + 3][Dn - 1 + 3];
  Un[1] = rule[Pn + 3][Dn - 1 + 3];
  Un[2] = rule[Pn - 1 + 3][Dn + 3];
  Un[3] = rule[Pn + 3][Dn + 3];
  if (PF[0] <= DF[0]) // 求小
    UF[0] = PF[0];
  else
    UF[0] = DF[0];
  if (PF[1] <= DF[0])
    UF[1] = PF[1];
  else
    UF[1] = DF[0];
  if (PF[0] <= DF[1])
    UF[2] = PF[0];
  else
    UF[2] = DF[1];
  if (PF[1] <= DF[1])
    UF[3] = PF[1];
  else
    UF[3] = DF[1];
  /*同隶属函数输出语言值求大*/
  if (Un[0] == Un[1]) {
    if (UF[0] > UF[1])
      UF[1] = 0;
    else
      UF[0] = 0;
  }
  if (Un[0] == Un[2]) {
    if (UF[0] > UF[2])
      UF[2] = 0;
    else
      UF[0] = 0;
  }
  if (Un[0] == Un[3]) {
    if (UF[0] > UF[3])
      UF[3] = 0;
    else
      UF[0] = 0;
  }
  if (Un[1] == Un[2]) {
    if (UF[1] > UF[2])
      UF[2] = 0;
    else
      UF[1] = 0;
  }
  if (Un[1] == Un[3]) {
    if (UF[1] > UF[3])
      UF[3] = 0;
    else
      UF[1] = 0;
  }
  if (Un[2] == Un[3]) {
    if (UF[2] > UF[3])
      UF[3] = 0;
    else
      UF[2] = 0;
  }
  t1 = UF[0] * UFF[Un[0]];
  t2 = UF[1] * UFF[Un[1]];
  t3 = UF[2] * UFF[Un[2]];
  t4 = UF[3] * UFF[Un[3]];
  temp1 = t1 + t2 + t3 + t4;
  temp2 = UF[0] + UF[1] + UF[2] + UF[3]; // 模糊量输出
  U = temp1 / temp2;

  // Uart_send(D*100);

  // Findline.kd1=U*0.01;
  U = U * 1;
  return U;
}
