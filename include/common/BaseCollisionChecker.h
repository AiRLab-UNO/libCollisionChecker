//
// Created by redwan on 1/7/26.
//

#ifndef MOTIONPLANNERBENCHMARK_BASECOLLISIONCHECKER_H
#define MOTIONPLANNERBENCHMARK_BASECOLLISIONCHECKER_H
#include <vector>
#include <array>
#include <memory>
#include <Eigen/Dense>
#include "ParamManager.h"
class BaseCollisionChecker;
using CCPtr = std::shared_ptr<BaseCollisionChecker>;

class BaseCollisionChecker: public std::enable_shared_from_this<BaseCollisionChecker>{
public:
    BaseCollisionChecker(const ParamPtr& pm):pm_(pm){

    }
    virtual bool isCollision(const std::vector<Eigen::VectorXd>&trajectory) const = 0;
    CCPtr getSharedPtr(){
        return shared_from_this();
    }

protected:
    ParamPtr pm_;


};


#endif //MOTIONPLANNERBENCHMARK_BASECOLLISIONCHECKER_H
