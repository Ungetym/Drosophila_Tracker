#ifndef SIMPLE_TRACKER_H
#define SIMPLE_TRACKER_H

#include <QObject>
#include <QDir>
#include <QCollator>
#include <iostream>

#include "basic_calc.h"
#include "consts.h"
#include "contour_extractor.h"
#include "system_state.h"

class Simple_Tracker: public QObject{
    Q_OBJECT

public:
    Simple_Tracker(System_State* state);

public slots:
    //extracts collisions of all sequences found in seq_dir and saves these collisions to coll_dir
    void track();

private:

    System_State* state;

    /////////////////////////////      simple tracking     ///////////////////////////////////////

    void processDir(const QString& dir_name);

    void easyTrack();
    //set numbers of active larvae per target using graph traversals
    void setLarvaeNumber(target* tgt, int* counter, bool forward);
    void setLarvaeNumbers();

    ///////////////////////////      collision extraction    /////////////////////////////////////

    //extracts collisions from tracked targets and saves them as separate image sequences (grey value + binary)
    void extractCollisions(const QString& dir_name);

    //inserts target into collision sequence
    void insert(target* tgt, collision* col);
    //traverse tracked target 'graph' in order to extract collisions
    void follow_path(target* tgt, bool forward, bool in_col, std::vector<collision>* collisions);

    ///////////////////////////      helper functions        /////////////////////////////////////

    //helper returns intersection rect of rect_2 in rect_1 normalzed to rect_1 and rect_2 coordinates
    std::pair<cv::Rect,cv::Rect> rectIntersection(cv::Rect rect_1, cv::Rect rect_2);
    //returns number of overlapping pixels of two contours
    int contourOverlap(contour& ct_1, contour& ct_2);

signals:
    void log(QString msg, int type);
    void progress(float val);
};

#endif // SIMPLE_TRACKER_H
