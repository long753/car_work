// #include "../SmartCar.hpp"
#include "RescueDetection.h"

void Rescue::reset(void) {
    carStoping = false;
    carExitting = false;
    step = Step::None;
    counterSession = 0;         // 图像场次计数器
    counterRec = 0;             // 标志检测计数器
    lastPointsEdgeLeft.clear(); // 记录上一场边缘点集（丢失边）
    lastPointsEdgeRight.clear();
    counterExit = 0;
    counterImmunity = 0;
  }

bool Rescue::process(Findline &track, vector<PredictResult> predict) {
    _pointNearCone = mpoint(0, 0);
    _distance = 0;
    pointConeLeft.clear();
    pointConeRight.clear();
    levelCones = 0;
    indexDebug = 0;

    switch (step) {
      case Step::None: //[01] 标志检测
        if (CarParams->rescue_key&&!again){//对无法看到牌子的特殊情况的处理，默认关闭
          searchCones(predict); // 搜索赛道左右两边锥桶
          if (pointConeLeft.size() >=CarParams->rescue_conenum &&pointConeRight.size()>=CarParams->rescue_conenum){
            float correlationLeft = calculateCorrelation(track.pointsEdgeLeft);
            float correlationRight = calculateCorrelation(track.pointsEdgeRight);
            std::cout<<"left"<<correlationLeft<<" right:"<<correlationRight<<std::endl;
            if (fabs(correlationLeft)>0.97&&fabs(correlationRight)>0.97){
              if (!CarParams->rescue_direction){
                step = Step::Enable; // 使能
                entryLeft = true;
                again = true;
                std::cout<<"rescue_dir为false,标志为patient/tumble"<<std::endl;
              }else{
                step = Step::Enable; // 使能
                entryLeft = false;
                again = true;
                std::cout<<"rescue_dir为true,标志为evil/thief"<<std::endl;
              }
              std::cout<<"根据相关系数进入状态"<<std::endl;
              break;
            }
       
          }
        }

        if ((counterImmunity > 200 && again) ||
            (counterImmunity > 2 && !again)) {//官方给了二次进入，目前用不到，again为false
          for (size_t i = 0; i < predict.size(); i++) {
            if (predict[i].type == LABEL_TUMBLE ||predict[i].type ==LABEL_PATIENT) // 平民标志检测
            {
              std::cout<<"成功识别tumble/patient"<<std::endl;
              counterRec++;
              break;
            }
            if (predict[i].type == LABEL_EVIL || predict[i].type ==LABEL_THIEF ) // 恐怖分子标志检测
            {
              std::cout<<"成功识别evil/thief"<<std::endl;
              counterExit++;
              break;
            }
          }
          // else{
          //     counterRec = 0;
          // }

          if (counterRec || counterExit) {
            counterSession++;
            if (CarParams->rescue_wrong_assert){
                step = Step::Enable; // 使能
                std::cout<<"看到标志后强制进入状态,false为平民左打,否则右打"<<endl;
                entryLeft = !CarParams->rescue_direction;
                counterRec = 0;
                counterExit = 0;
                counterSession = 0;
                return true;
            }
            if (counterRec >= CarParams->rescue_labelframe && counterSession <= 8) {
              step = Step::Enable; // 使能
              std::cout<<"进入Enable----left"<<endl;
              entryLeft = true;
              counterRec = 0;
              counterExit = 0;
              counterSession = 0;
              return true;
            } else if (counterExit >= CarParams->rescue_labelframe && counterSession <= 8) {
              step = Step::Enable; // 使能
              std::cout<<"进入Enable----right"<<endl;
              entryLeft = false;
              counterRec = 0;
              counterExit = 0;
              counterSession = 0;
              return true;
            } else if (counterSession > 8) {
              counterRec = 0;
              counterSession = 0;
            }
          }
        } 
        else{
          counterImmunity++;
        }
        break;
    
      case Step::Enable: //[02] 使能
      {
        counterExit++;
        if (counterExit > 250) // 超时退出
        {
          reset();
          std::cout<<"Enable超时退出"<<std::endl;
          return false;
        }
        enter_row =  40;
        
        searchCones(predict); // 搜索赛道左右两边锥桶
        if (entryLeft)        // 左入库
        {
          _pointNearCone = getConeLeftDown(track.pointsEdgeLeft,
                                          pointConeLeft); 
          if (_pointNearCone.row >enter_row) // 当车辆开始靠近右边锥桶：准备入库
          {
            counterRec++;
            if (counterRec >= 1) {
              step = Step::Enter; // 进站使能
              angle = angle_;
              std::cout<<"进入Enter_left"<<endl;
              counterRec = 0;
              counterSession = 0;
              counterExit = 0;
              pathsEdgeLeft.clear();
              pathsEdgeRight.clear();
              break;
            }
          }
        } 
        else // 右入库
        {
          _pointNearCone = getConeRightDown(track.pointsEdgeRight,
                                            pointConeRight); // 搜索左下锥桶
          if (_pointNearCone.row >enter_row) // 当车辆开始靠近右边锥桶：准备入库，0.4
          {
            counterRec++;
            if (counterRec >= 1) {
              step = Step::Enter; // 进站使能
              std::cout<<"进入Enter_right"<<endl;
              counterRec = 0;
              counterSession = 0;
              counterExit = 0;
              pathsEdgeLeft.clear();
              pathsEdgeRight.clear();
              break;

            }
          }
        }
        break;
      }
      case Step::Enter: //[03] 入库使能
      {
        counterSession++; // 屏蔽期:防止提前入库
        std::cout<<"cnt"<<counterSession<<endl;

        vector<PredictResult> resultsObs; // 锥桶AI检测数据
        for (int i = 0; i < predict.size(); i++){
            if (predict[i].type == LABEL_CONE )
                resultsObs.push_back(predict[i]);
        }
        if (resultsObs.size() <= 0){
            std::cout<<"未找到锥桶"<<std::endl;
            break;
        }

        int rowMax = 0; 
        for (int i = 0; i < resultsObs.size(); i++){//最上面的
          int area = resultsObs[i].height + resultsObs[i].y;
          if (area > rowMax && area <ROWSIMAGE){
            rowMax = area;
          }
        }
        std::cout<<" rowMax:"<<rowMax<<",last:"<<Rowmax_last<<std::endl;
        std::cout<<"angle"<<angle_<<" angle_init"<<angle<<" abs:"<<calAnglediff(angle_,angle)<<std::endl;
        if (Rowmax_last-rowMax >50) enter_flag_Rowmax = true;
        if (counterSession > 10 && enter_flag_Rowmax ) {//屏蔽结束
            counterExit++;
            if (counterExit > 1) {
              counterExit = 0;
              counterRec = 0;
              counterSession = 0;
              angle = angle_;
              step = Step::Cruise; // 停车使能
              std::cout<<"rowMax满足条件,进入cruise "<<std::endl;
              break;
            }
        }
        Rowmax_last = rowMax;
        break;
      }
      case Step::Cruise:{
        counterSession++;
        if (counterSession < CarParams->rescue_enter_count){
          break;
        }
        std::cout<<"angle"<<angle_<<" angle_init"<<angle<<" abs:"<<calAnglediff(angle_,angle)<<std::endl;
        if (calAnglediff(angle_,angle)>CarParams->rescue_angle_diff ){
            counterRec++;
            if (counterRec>CarParams->rescue_enterstop_count){
              step = Step::Stop; // 停车使能
              std::cout<<"Cruise->Stop"<<std::endl;
              counterSession = 0;
              counterRec = 0;
              break;
            }
        }
        if (entryLeft){ // 左入库
          mpoint start = mpoint(ROWSIMAGE - 40, COLSIMAGE - 1);
          mpoint end = mpoint(CarParams->rescue_endpoint_row, 0);
          mpoint middle =
              mpoint((start.row + end.row) * 0.4, (start.col + end.col) * 0.6);
          vector<mpoint> input = {start, middle, end};
          track.pointsEdgeLeft.resize(0); // 删除错误路线
          track.pointsEdgeRight = Bezier(0.001, input); // 补线
          for (int i = (CarParams->rescue_endpoint_row); i < (ROWSIMAGE - 40); i++){
            track.pointsEdgeLeft.push_back(mpoint(i,5));
          }
        } else // 右入库
        {
          mpoint start = mpoint(ROWSIMAGE - 40, 0);
          mpoint end = mpoint(CarParams->rescue_endpoint_row, COLSIMAGE - 1);
          mpoint middle =
              mpoint((start.row + end.row) * 0.4, (start.col + end.col) * 0.6);
          vector<mpoint> input = {start, middle, end};
          track.pointsEdgeRight.resize(0); // 删除错误路线
          track.pointsEdgeLeft = Bezier(0.001, input); // 补线
          for (int i = (CarParams->rescue_endpoint_row); i < (ROWSIMAGE - 40); i++){
            track.pointsEdgeRight.push_back(mpoint(i,314));
          }
        }
        break;
      }

      case Step::Stop: //[05] 停车使能
      {
        carStoping = true;
        counterRec++;
        if (counterRec > 0) // 停车
        {
          carStoping = false;
          carExitting = true;
          std::cout<<"Stop--->>>Exit"<<std::endl;
          step = Step::Exit; // 出站使能
          counterRec = 0;
        }
        break;
      }

      case Step::Exit: //[06] 出站使能
      {
        carExitting = true;
        std::cout<<"leftsize:"<<track.pointsEdgeLeft.size()<<" rightsize:"<<track.pointsEdgeRight.size()<<std::endl;
        std::cout<<"angle"<<angle_<<" angle_init"<<angle<<" abs:"<<calAnglediff(angle_,angle)<<std::endl;

        if (track.pointsEdgeLeft.size()>ROWSIMAGE/2 && track.pointsEdgeRight.size()>ROWSIMAGE/2){
          counterExit++;
          if (counterExit > 1&&calAnglediff(angle_,angle) <20) {
            step = Step::None; // 出站完成
            std::cout<<"出站完成"<<std::endl;
            carExitting = false;
            again = true; // 第二次进入救援区标志
            reset();
            break;
          }
        }
        if (entryLeft){ // 左出库
          mpoint start = mpoint(ROWSIMAGE - 40, COLSIMAGE - 1);
          mpoint end = mpoint(CarParams->rescue_endpoint_row, 0);
          mpoint middle =
              mpoint((start.row + end.row) * 0.4, (start.col + end.col) * 0.6);
          vector<mpoint> input = {start, middle, end};
          track.pointsEdgeRight = Bezier(0.001, input); // 补线
          track.pointsEdgeLeft.resize(0); // 删除错误路线
          for (int i = (CarParams->rescue_endpoint_row); i < (ROWSIMAGE - 40); i++){
              track.pointsEdgeLeft.push_back(mpoint(i,5));
          }
        } 
        else if (!entryLeft ){ // 右出库
          mpoint start = mpoint(ROWSIMAGE - 40, 0);
          mpoint end = mpoint(CarParams->rescue_endpoint_row, COLSIMAGE - 1);
          mpoint middle =
              mpoint((start.row + end.row) * 0.4, (start.col + end.col) * 0.6);
          vector<mpoint> input = {start, middle, end};
          track.pointsEdgeLeft = Bezier(0.001, input); // 补线
          track.pointsEdgeRight.resize(0); // 删除错误路线
          for (int i = (CarParams->rescue_endpoint_row); i < (ROWSIMAGE - 40); i++){
              track.pointsEdgeRight.push_back(mpoint(i,314));
          }
        }
        break;
      }
    }

    if (step == Step::None) // 返回控制模式标志
      return false;
    else
      return true;
  }

void Rescue::drawImage(Findline track, Mat &image) {
    // 赛道边缘
    for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++) {
      circle(image, Point(track.pointsEdgeLeft[i].col, track.pointsEdgeLeft[i].row),
             1, Scalar(0, 255, 0), -1); // 绿色点
    }
    for (size_t i = 0; i < track.pointsEdgeRight.size(); i++) {
      circle(image,
             Point(track.pointsEdgeRight[i].col, track.pointsEdgeRight[i].row), 1,
             Scalar(0, 255, 255), -1); // 黄色点
    }

    // 入库状态
    string state = "None";
    switch (step) {
    case Step::Enable:
      state = "Enable";
      break;
    case Step::Enter:
      state = "Enter";
      break;
    case Step::Cruise:
      state = "Cruise";
      break;
    case Step::Stop:
      state = "Stop";
      break;
    case Step::Exit:
      state = "Exit";
      break;
    }
    if (entryLeft) {
      // 绘制锥桶坐标
      for (size_t i = 0; i < pointConeLeft.size(); i++) {
        circle(image, Point(pointConeLeft[i].col, pointConeLeft[i].row), 2,
               Scalar(92, 92, 205), -1); // 锥桶坐标：红色
      }
      putText(image, "[3] RESCUE - LEFT", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    } else {
      // 绘制锥桶坐标
      for (size_t i = 0; i < pointConeRight.size(); i++) {
        circle(image, Point(pointConeRight[i].col, pointConeRight[i].row), 2,
               Scalar(92, 92, 205), -1); // 锥桶坐标：红色
      }
      putText(image, "[3] RESCUE - RIGHT", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    putText(image, state, Point(COLSIMAGE / 2 - 10, 30),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

    putText(image, to_string(_distance), Point(COLSIMAGE / 2 - 15, 40),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
            CV_AA); // 显示锥桶距离
    if (_pointNearCone.row > 0)
      circle(image, Point(_pointNearCone.col, _pointNearCone.row), 5,
             Scalar(200, 200, 200), -1);

    if (levelCones > 0)
      line(image, Point(0, levelCones), Point(image.cols, levelCones),
           Scalar(255, 255, 255), 1);

    putText(image, to_string(indexDebug),
            Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX,
            0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
  }

void Rescue::searchCones(vector<PredictResult> predict) {
    pointConeLeft.clear();
    pointConeRight.clear();
    for (size_t i = 0; i < predict.size(); i++) {
      if (predict[i].type == LABEL_CONE) // 锥桶检测
      {
        if ((predict[i].x + predict[i].width / 2) < COLSIMAGE / 2)
          pointConeLeft.push_back(mpoint(predict[i].y + predict[i].height,
                                        predict[i].x + predict[i].width));
        else
          pointConeRight.push_back(
              mpoint(predict[i].y + predict[i].height, predict[i].x));
      }
    }
  }
  void Rescue::assert_conenum(vector<PredictResult> predict){
    pointConeLeft.clear();
    for (size_t i = 0; i < predict.size(); i++) {
      if (predict[i].type == LABEL_CONE) // 锥桶检测
      {
          pointConeLeft.push_back(mpoint(predict[i].y + predict[i].height,
            predict[i].x + predict[i].width));
      }
    }
  } 


mpoint Rescue::getConeLeftDown(vector<mpoint> pointsEdgeLeft,
                        vector<mpoint> pointsCone) {
    mpoint point(0, 0);
    double disMin = 60; // 右边缘锥桶离赛道左边缘最小距离

    if (pointsCone.size() <= 0 || pointsEdgeLeft.size() < 10)
      return point;

    for (size_t i = 0; i < pointsCone.size(); i++) {
      if (pointsEdgeLeft[pointsEdgeLeft.size() - 1].row < pointsCone[i].row) {
        int row = pointsEdgeLeft[0].row - pointsCone[i].row;
        if (row > 0 && row < pointsEdgeLeft.size()) {
          int dis = pointsEdgeLeft[row].col - pointsCone[i].col;
          if (dis < disMin && pointsCone[i].row > ROWSIMAGE / 4 &&//ROWSIMAGE / 4
              pointsCone[i].row > point.row) {
            point = pointsCone[i];
            _distance = dis;
          }
        }
      }
    }
    return point;
  }

mpoint Rescue::getConeRightDown(vector<mpoint> pointsEdgeRight,
                         vector<mpoint> pointsCone) {
    mpoint point(0, 0);
    double disMin = 60; // 右边缘锥桶离赛道左边缘最小距离

    if (pointsCone.size() <= 0 || pointsEdgeRight.size() < 10)
      return point;

    for (size_t i = 0; i < pointsCone.size(); i++) {
      if (pointsEdgeRight[pointsEdgeRight.size() - 1].row < pointsCone[i].row) {
        int row = pointsEdgeRight[0].row - pointsCone[i].row;
        if (row > 0 && row < pointsEdgeRight.size()) {
          int dis = pointsCone[i].col - pointsEdgeRight[row].col;
          if (dis < disMin && pointsCone[i].row > ROWSIMAGE / 4 &&
              pointsCone[i].row > point.row) {
            point = pointsCone[i];
            _distance = dis;
          }
        }
      }
    }

    return point;
  }

vector<mpoint> Rescue::predictEdgeRight(vector<mpoint> &pointsEdgeLeft) {
    int offset = 120; // 右边缘平移尺度
    vector<mpoint> pointsEdgeRight;
    if (pointsEdgeLeft.size() < 3)
      return pointsEdgeRight;

    // Start
    Point2d startIpm = ipm.homography(
        Point2d(pointsEdgeLeft[0].col, pointsEdgeLeft[0].row)); // 透视变换
    Point2d prefictRight = Point2d(startIpm.x + offset, startIpm.y);
    Point2d startIipm = ipm.homographyInv(prefictRight); // 反透视变换
    mpoint startPoint = mpoint(startIipm.y, startIipm.x);

    // Middle
    Point2d middleIpm = ipm.homography(
        Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].col,
                pointsEdgeLeft[pointsEdgeLeft.size() / 2].row)); // 透视变换
    prefictRight = Point2d(middleIpm.x + offset, middleIpm.y);
    Point2d middleIipm = ipm.homographyInv(prefictRight); // 反透视变换
    mpoint midPoint = mpoint(middleIipm.y, middleIipm.x);   // 补线中点

    // End
    Point2d endIpm = ipm.homography(
        Point2d(pointsEdgeLeft[pointsEdgeLeft.size() - 1].col,
                pointsEdgeLeft[pointsEdgeLeft.size() - 1].row)); // 透视变换
    prefictRight = Point2d(endIpm.x + offset, endIpm.y);
    Point2d endtIipm = ipm.homographyInv(prefictRight); // 反透视变换
    mpoint endPoint = mpoint(endtIipm.y, endtIipm.x);

    // 补线
    vector<mpoint> input = {startPoint, midPoint, endPoint};
    vector<mpoint> repair = Bezier(0.05, input);

    for (size_t i = 0; i < repair.size(); i++) {
      if (repair[i].row >= ROWSIMAGE)
        repair[i].row = ROWSIMAGE - 1;

      else if (repair[i].row < 0)
        repair[i].row = 0;

      else if (repair[i].col >= COLSIMAGE)
        repair[i].col = COLSIMAGE - 1;
      else if (repair[i].col < 0)
        repair[i].col = 0;

      pointsEdgeRight.push_back(repair[i]);
    }

    return pointsEdgeRight;
  }

vector<mpoint> Rescue::predictEdgeLeft(vector<mpoint> &pointsEdgeRight) {
    int offset = 120; // 右边缘平移尺度
    vector<mpoint> pointsEdgeLeft;
    if (pointsEdgeRight.size() < 3)
      return pointsEdgeLeft;

    // Start
    Point2d startIpm = ipm.homography(
        Point2d(pointsEdgeRight[0].col, pointsEdgeRight[0].row)); // 透视变换
    Point2d prefictLeft = Point2d(startIpm.x - offset, startIpm.y);
    Point2d startIipm = ipm.homographyInv(prefictLeft); // 反透视变换
    mpoint startPoint = mpoint(startIipm.y, startIipm.x);

    // Middle
    Point2d middleIpm = ipm.homography(
        Point2d(pointsEdgeRight[pointsEdgeRight.size() / 2].col,
                pointsEdgeRight[pointsEdgeRight.size() / 2].row)); // 透视变换
    prefictLeft = Point2d(middleIpm.x - offset, middleIpm.y);
    Point2d middleIipm = ipm.homographyInv(prefictLeft); // 反透视变换
    mpoint midPoint = mpoint(middleIipm.y, middleIipm.x);  // 补线中点

    // End
    Point2d endIpm = ipm.homography(
        Point2d(pointsEdgeRight[pointsEdgeRight.size() - 1].col,
                pointsEdgeRight[pointsEdgeRight.size() - 1].row)); // 透视变换
    prefictLeft = Point2d(endIpm.x - offset, endIpm.y);
    Point2d endtIipm = ipm.homographyInv(prefictLeft); // 反透视变换
    mpoint endPoint = mpoint(endtIipm.y, endtIipm.x);

    // 补线

    vector<mpoint> input = {startPoint, midPoint, endPoint};
    vector<mpoint> repair = Bezier(0.05, input);

    for (size_t i = 0; i < repair.size(); i++) {
      if (repair[i].row >= ROWSIMAGE)
        repair[i].row = ROWSIMAGE - 1;

      else if (repair[i].row < 0)
        repair[i].row = 0;

      else if (repair[i].col >= COLSIMAGE)
        repair[i].col = COLSIMAGE - 1;
      else if (repair[i].col < 0)
        repair[i].col = 0;

      pointsEdgeLeft.push_back(repair[i]);
    }

    return pointsEdgeLeft;
  }


  void Rescue::pointsSortForY(vector<mpoint> &points) {
    int n = points.size();
    bool flag = true;

    for (int i = 0; i < n - 1 && flag; i++) {
      flag = false;
      for (int j = 0; j < n - i - 1; j++) {
        if (points[j].col > points[j + 1].col) {
          mpoint temp = points[j];
          points[j] = points[j + 1];
          points[j + 1] = temp;
          flag =
              true; // 每次循环i有修改，这里为true
                    // 如果跑了一次I没有发生交换的情况，说明已经排序完成，不需要再跑后面的i
        }
      }
    }
  }
