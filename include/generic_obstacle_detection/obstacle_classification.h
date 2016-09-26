#ifndef OBSTACLE_CLASSIFICATION_H
#define OBSTACLE_CLASSIFICATION_H

namespace csapex
{

enum class ObstacleClassification {
    UNKNOWN = 0,
    FLOOR = 1,
    UNSPECIFIED_OBSTACLE = 2,
    OBJECT = 3,
    LOW_OBSTACLE = 4,
    OVERHANGING_OBSTACLE = 5,
    WALL = 6,
    CEILING = 7,

    ARTIFACT = 9
};

}

#endif // OBSTACLE_CLASSIFICATION_H
