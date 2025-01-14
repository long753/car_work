##pragma once 
#include "common.hpp"
#include "../element/Findline.h"
#include "../include/detection.hpp"

class Ring
{
public:
    bool process(Findline &track, vector<PredictResult> predict);
    void drawImage(Mat &image,Findline track);

private:
 
 
};