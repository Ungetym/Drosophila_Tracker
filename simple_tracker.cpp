#include "simple_tracker.h"

using namespace std;
using namespace cv;

Simple_Tracker::Simple_Tracker(System_State *state){
    this->state=state;
    this->setObjectName("Simple Tracker");
}

void Simple_Tracker::track(){
    //check if input dir exists
    QDir dir(state->io.seq_dir);
    if(!dir.exists()){
        ERROR("Input dir "+state->io.seq_dir+" does not exist.");
        return;
    }
    //check if input dir contains subdirs
    QStringList subdirs = dir.entryList(QDir::NoDotAndDotDot|QDir::AllEntries);
    if(subdirs.size()==0){
        ERROR("Could not find sequences in input dir "+state->io.seq_dir);
        return;
    }

    //check if output dir exists
    QDir dir_coll(state->io.coll_dir);
    if(!dir_coll.exists()){
        ERROR("Collisions dir "+state->io.coll_dir+" does not exist.");
        return;
    }

    STATUS("Start contour extraction.");

    for(int i=0;i<subdirs.size();i++){
        QString& subdir = subdirs[i];
        //clear current data
        state->simple.tracked_targets.clear();
        state->simple.last_contours.clear();
        state->simple.current_contours.clear();
        state->simple.file_list.clear();

        QString path = state->io.seq_dir+"/"+subdir;

        //track targets
        processDir(path);

        //after tracking set numbers of active larvae per target
        setLarvaeNumbers();

        //extract collisions by following the tracks based on larvae numbers
        extractCollisions(path);

        //progress feedback for gui
        emit progress((float)(i+1)/(float)(subdirs.size()));
    }

    STATUS("Contour extraction done.");
}


//////////////////////////////////////////////////      simple tracking     ////////////////////////////////////////////////////////////

void Simple_Tracker::processDir(const QString& dir_name){
    //get list of relevant files in given dir
    QDir dir(dir_name);
    QStringList filters;
    filters << "*.tiff" << "*.tif" << "*.png";
    state->simple.file_list = dir.entryList(filters);

    //sort filenames in natural way
    QCollator col;
    col.setNumericMode(true);
    col.setCaseSensitivity(Qt::CaseInsensitive);
    std::sort(state->simple.file_list.begin(), state->simple.file_list.end(), [&](const QString& a, const QString& b) {
        return col.compare(a, b) < 0;
    });

    Mat image;
    bool first_frame = true;
    string frame_name;
    contour_extractor ct_extractor;
    float min_area =500.0;
    float max_area =20000.0;

    for(auto const& file : state->simple.file_list)
    {
        frame_name = dir_name.toStdString() + "/" + file.toStdString();

        //read current frame
        image = cv::imread(frame_name, CV_LOAD_IMAGE_COLOR);
        //cv::resize(image, image, cv::Size(), 2.0,2.0);

        std::vector<vector<Point>> detected_contours;
        if(first_frame){
            //init background
            ct_extractor.initBackground(&image);
        }

        //detect contours
        detected_contours = ct_extractor.extractContours(&image, &state->simple.current_binary, min_area, max_area);

        for(size_t i=0;i<detected_contours.size();i++){
            vector<Point>& ct = detected_contours[i];
            contour new_ct;
            new_ct.contour_points = ct;
            Point max_pos = ct[0];
            Point min_pos = ct[1];

            for(Point& p : ct){
                min_pos.x=min(p.x,min_pos.x);
                max_pos.x=max(p.x,max_pos.x);
                min_pos.y=min(p.y,min_pos.y);
                max_pos.y=max(p.y,max_pos.y);
            }
            new_ct.pos_in_binary = Rect(min_pos.x,min_pos.y,max_pos.x-min_pos.x,max_pos.y-min_pos.y);

            Mat ct_bin = cv::Mat::zeros(state->simple.current_binary.rows,state->simple.current_binary.cols,state->simple.current_binary.type());
            drawContours(ct_bin,detected_contours,i,Scalar(255,255,255),-1);
            new_ct.binary = ct_bin(new_ct.pos_in_binary).clone();

            state->simple.current_contours.push_back(new_ct);
        }

        //associate current contours with last ones by checking for overlaps
        easyTrack();

        //set "last" data
        state->simple.last_contours=state->simple.current_contours;
        state->simple.current_contours.clear();
        state->simple.last_binary = state->simple.current_binary;

        //calculate min_area and max_area of allowed larvae contours
        if(first_frame){
            vector<int> areas;
            for(target& tgt : state->simple.tracked_targets.back()){
                areas.push_back(cv::countNonZero(tgt.ct.binary));
            }
            std::sort(areas.begin(),areas.end());
            min_area = 0.3*areas[areas.size()/2];
            max_area=33.3f*min_area;
            first_frame=false;
        }

    }
}

void Simple_Tracker::easyTrack(){
    if( state->simple.tracked_targets.size()==0){
        vector<target> current_targets;
        for(contour& ct : state->simple.current_contours){
            target new_tgt;
            new_tgt.ct = ct;
            new_tgt.ids.push_back(current_targets.size());
            new_tgt.frame_nr=0;
            new_tgt.min_larvae=1;
            new_tgt.max_larvae=1;
            current_targets.push_back(new_tgt);
        }
         state->simple.tracked_targets.push_back(current_targets);
    }
    else{
        vector<target> current_targets;
         state->simple.tracked_targets.push_back(current_targets);

        for(contour& ct : state->simple.current_contours){
            target new_tgt;
            new_tgt.ct = ct;
            new_tgt.frame_nr= state->simple.tracked_targets.size()-1;
            new_tgt.min_larvae=1;
            new_tgt.max_larvae=1;
             state->simple.tracked_targets.back().push_back(new_tgt);
        }

        for(size_t j=0; j<state->simple.current_contours.size(); j++){//necessary to allow pointers to vector<target> elements..
            contour& ct = state->simple.current_contours[j];

            for(size_t i=0; i< state->simple.tracked_targets[ state->simple.tracked_targets.size()-2].size(); i++){
                target* tgt = &( state->simple.tracked_targets[ state->simple.tracked_targets.size()-2][i]);
                //check if contours overlap
                if(contourOverlap(ct,tgt->ct)>100){
                    vector<int>* ids = & state->simple.tracked_targets.back()[j].ids;
                    for(int id : tgt->ids){
                        if (std::find(ids->begin(), ids->end(), id) == ids->end()) {
                            ids->push_back(id);
                        }
                    }
                     state->simple.tracked_targets.back()[j].pred.push_back(i);
                    tgt->succ.push_back(j);
                }
            }

        }
    }
}

void Simple_Tracker::setLarvaeNumber(target* tgt, int* counter, bool forward){

    int old_max = tgt->max_larvae;
    int old_min = tgt->min_larvae;

    if(forward){
        if(tgt->pred.size()!=0){
            tgt->min_larvae=0;
            tgt->max_larvae=0;

            vector<int> unique_identities;

            for(int pred_idx : tgt->pred){
                target* pred = &state->simple.tracked_targets[tgt->frame_nr-1][pred_idx];
                if(pred->succ.size()==1){
                    tgt->min_larvae += pred->min_larvae;
                    tgt->max_larvae += pred->max_larvae;
                }
                else{
                    int added_mins=0;
                    for(int succ_idx : pred->succ){
                        added_mins+= state->simple.tracked_targets[tgt->frame_nr][succ_idx].min_larvae;
                    }
                    tgt->max_larvae += pred->max_larvae-added_mins;
                    tgt->min_larvae = max(old_min,1);
                }

                for(int id : tgt->ids){
                    if (std::find(unique_identities.begin(), unique_identities.end(), id) == unique_identities.end()) {
                        unique_identities.push_back(id);
                    }
                }
            }
            tgt->min_larvae = max(tgt->min_larvae,old_min);
            tgt->max_larvae = min(tgt->max_larvae, (int)unique_identities.size());
            tgt->max_larvae = max(tgt->max_larvae,tgt->min_larvae);
        }
    }
    else{
        if(tgt->succ.size()!=0){
            if(tgt->succ.size()==1){
                target* succ = &state->simple.tracked_targets[tgt->frame_nr+1][tgt->succ[0]];
                if(succ->pred.size()==1){
                    tgt->min_larvae = succ->min_larvae;
                    tgt->max_larvae = succ->max_larvae;
                }
            }
            else{
                tgt->min_larvae=max((int)tgt->succ.size(),tgt->min_larvae);

                int min_count=0;
                for(int succ_idx : tgt->succ){
                    target* succ = &state->simple.tracked_targets[tgt->frame_nr+1][succ_idx];
                    if(succ->pred.size()==1){
                        min_count += succ->min_larvae;
                    }
                    else{
                        min_count++;
                    }
                }
                tgt->min_larvae=max(tgt->min_larvae,min_count);
                tgt->max_larvae=max(tgt->min_larvae,tgt->max_larvae);
            }
        }
    }

    if(old_max-tgt->max_larvae!=0 || old_min-tgt->min_larvae!=0){
        (*counter)++;
    }
}

void Simple_Tracker::setLarvaeNumbers(){
    //set larvae numbers by tree traversal
    int change_counter=1;//numbers of changed larvae counts
    bool forward = true;
    while(change_counter>0){
        change_counter=0;
        for(size_t frame=0; frame< state->simple.tracked_targets.size(); frame++){
            int realframe = frame;
            if(!forward){
                realframe =  state->simple.tracked_targets.size()-1-frame;
            }
            vector<target>& current_targets =  state->simple.tracked_targets[realframe];
            for(target& tgt : current_targets){
                setLarvaeNumber(&tgt, &change_counter, forward);
            }
        }
        forward=!forward;
    }
}

//////////////////////////////////////////////////      collision extraction    ////////////////////////////////////////////////////////

void Simple_Tracker::insert(target* tgt, collision* col){
    if(col->targets.size()==0){
        col->start_frame=tgt->frame_nr;
        col->end_frame=tgt->frame_nr;
        vector<target> new_vector;
        col->targets.push_back(new_vector);
    }
    else{
        if(tgt->frame_nr<col->start_frame){
            col->start_frame=tgt->frame_nr;
            vector<vector<target>> new_vector = {{*tgt}};
            col->targets.insert(col->targets.begin(),new_vector.begin(),new_vector.end());
            return;
        }
        if(tgt->frame_nr>col->end_frame){
            col->end_frame=tgt->frame_nr;
            vector<target> new_vector;
            col->targets.push_back(new_vector);
        }
    }
    col->targets[tgt->frame_nr-col->start_frame].push_back(*tgt);
}

void Simple_Tracker::follow_path(target* tgt, bool forward, bool in_col, vector<collision>* collisions){
    if(!in_col){
        if(!tgt->marked){
            tgt->marked=true;
            if(tgt->min_larvae>1){//first contour of collision
                //create new collision
                collision new_col;
                collisions->push_back(new_col);
                insert(tgt,&collisions->back());
                //follow succs and preds
                for(int succ_idx : tgt->succ){
                    follow_path(&state->simple.tracked_targets[tgt->frame_nr+1][succ_idx],true,true,collisions);
                }
                for(int pred_idx : tgt->pred){
                    follow_path(&state->simple.tracked_targets[tgt->frame_nr-1][pred_idx],false,true,collisions);
                }
            }
            else{//not part of collision, proceed with succ
                if(forward && tgt->succ.size()==1){
                    follow_path(&state->simple.tracked_targets[tgt->frame_nr+1][tgt->succ[0]],forward,false,collisions);
                }
            }
        }
    }
    else{//contour is part of last added collision
        if(tgt->max_larvae==1){//collision ends here
            tgt->marked=true;
            insert(tgt,&collisions->back());
        }
        else{
            if(!tgt->marked){
                tgt->marked=true;
                insert(tgt,&collisions->back());
                //follow succs and preds
                for(int succ_idx : tgt->succ){
                    follow_path(&state->simple.tracked_targets[tgt->frame_nr+1][succ_idx],true,true,collisions);
                }
                for(int pred_idx : tgt->pred){
                    follow_path(&state->simple.tracked_targets[tgt->frame_nr-1][pred_idx],false,true,collisions);
                }
            }
        }
    }
}

void Simple_Tracker::extractCollisions(const QString& dir_name)
{
    //get list of relevant files in given dir
    QDir dir(dir_name);
    QStringList filters;
    filters << "*.tiff" << "*.tif" << "*.png";
    QStringList file_list = dir.entryList(filters);

    //sort numbers natural order
    QCollator col;
    col.setNumericMode(true);
    col.setCaseSensitivity(Qt::CaseInsensitive);
    std::sort(file_list.begin(), file_list.end(), [&](const QString& a, const QString& b) {return col.compare(a, b) < 0;});

    //get collisions via target graph traversal
    vector<collision> collisions;
    for(size_t frame_nr=0; frame_nr<state->simple.tracked_targets.size(); frame_nr++){
        for(target& tgt : state->simple.tracked_targets[frame_nr]){
            follow_path(&tgt, true, false, &collisions);
        }
    }

    auto combineRect = [&](Rect& rect_1, Rect& rect_2){
        if(rect_1.x==-1){
            rect_1=rect_2;
            return;
        }

        int min_x = min(rect_1.x,rect_2.x);
        int max_x = max(rect_1.x+rect_1.width,rect_2.x+rect_2.width);
        int min_y = min(rect_1.y,rect_2.y);
        int max_y = max(rect_1.y+rect_1.height,rect_2.y+rect_2.height);

        rect_1.x=min_x;
        rect_1.y=min_y;
        rect_1.width=max_x-min_x;
        rect_1.height=max_y-min_y;
    };

    //build and save image sequences containing only one collision
    for(size_t coll_idx=0; coll_idx<collisions.size(); coll_idx++){
        collision& coll = collisions[coll_idx];

        //check if collision starts and ends with only single larvae
        bool separated_frames_available = true;
        for(target& tgt : coll.targets[0]){
            if(tgt.min_larvae>1){
                separated_frames_available=false;
            }
        }
        for(target& tgt : coll.targets.back()){
            if(tgt.min_larvae>1){
                separated_frames_available=false;
            }
        }

        if(separated_frames_available){//if no unsolved contours exist, save the collision
            int participating_larvae = 2;//number of larvae active in collision

            //find a bounding box containing all larvae of the collision
            Rect bounding_box;
            bounding_box.x=-1;
            for(vector<target>& frame_targets : coll.targets){//targets in current frame
                participating_larvae = max(participating_larvae,(int)frame_targets.size());
                for(target& tgt : frame_targets){
                    combineRect(bounding_box,tgt.ct.pos_in_binary);
                }
            }
            bounding_box.x = max(bounding_box.x-10,0);
            bounding_box.y = max(bounding_box.y-10,0);
            bounding_box.width = min(bounding_box.width+20,state->simple.last_binary.cols-bounding_box.x);
            bounding_box.height = min(bounding_box.height+20,state->simple.last_binary.rows-bounding_box.y);

            //create dir to save the sequence
            string sequence_name = dir_name.toStdString();
            int idx = sequence_name.find_last_of("/");
            sequence_name = sequence_name.substr(idx+1,sequence_name.size()-idx-1);
            string output_dir_str = state->io.coll_dir.toStdString()+"/collisions_"+to_string(participating_larvae);
            //check if exists, otherwise mkdir
            QDir output_qdir(QString::fromStdString(output_dir_str));
            if(!output_qdir.exists()){
                output_qdir.mkpath(".");
            }
            output_dir_str+="/"+sequence_name+"_"+to_string(coll_idx)+"/";
            //cut input images to bounding_box and build binary images to mask the cropped images
            Mat image;
            for(int frame=coll.start_frame; frame<coll.end_frame+1; frame++){
                string frame_name = dir_name.toStdString() + "/" + file_list[frame].toStdString();
                //read current frame
                image = cv::imread(frame_name, CV_LOAD_IMAGE_COLOR);
                //crop to bounding box
                cv::Mat cropped = image(bounding_box);
                //create the binary image
                cv::Mat binary = cv::Mat::zeros(cropped.rows, cropped.cols,cropped.type());
                Point offset(bounding_box.x,bounding_box.y);
                for(target& tgt : coll.targets[frame-coll.start_frame]){
                    vector<Point> contour;
                    for(Point p : tgt.ct.contour_points){
                        contour.push_back(p-offset);
                    }
                    vector<vector<Point>> contours = {contour};
                    cv::drawContours(binary,contours,0,cv::Scalar(255,255,255),-1);
                }
                //mask the cut input image by the binary image and save the masked image
                cropped = cropped&binary;
                if(frame==coll.start_frame){
                    QDir dir(QString::fromStdString(output_dir_str));
                    if (!dir.exists()) {
                        dir.mkpath(".");
                    }
                }
                cv::imwrite(output_dir_str+to_string(frame)+".tiff",cropped);
            }
        }
    }
}


//////////////////////////////////////////////////      helper functions        ////////////////////////////////////////////////////////

//helper returns intersection rect of rect_2 in rect_1 normalzed to rect_1 and rect_2 coordinates
pair<Rect,Rect> Simple_Tracker::rectIntersection(Rect rect_1, Rect rect_2){
    int x1 = rect_1.x;
    int x2 = x1+rect_1.width-1;
    int y1 = rect_1.y;
    int y2 = y1+rect_1.height-1;

    int a1 = rect_2.x;
    int a2 = a1+rect_2.width-1;
    int b1 = rect_2.y;
    int b2 = b1+rect_2.height-1;

    bool check1 = false;
    bool check2 = false;
    if((a1<=x1 && a2>x1)||(a1<x2&&a2>=x2)||(a1>x1 && a2<x2)){
        check1 = true;
    }
    if((b1<=y1 && b2>y1)||(b1<y2&&b2>=y2)||(b1>y1 && b2<y2)){
        check2 = true;
    }

    pair<Rect,Rect> result;
    if(check1&check2){
        int x1_new = (x1<=a1) ? a1 : x1;
        int x2_new = (x2<=a2) ? x2 : a2;
        int y1_new = (y1<=b1) ? b1 : y1;
        int y2_new = (y2<=b2) ? y2 : b2;

        result.first = Rect(x1_new-x1,y1_new-y1,x2_new-x1_new+1,y2_new-y1_new+1);
        result.second = Rect(x1_new-a1,y1_new-b1,x2_new-x1_new+1,y2_new-y1_new+1);
    }
    else{
        result.first = Rect(0,0,0,0);
        result.second = Rect(0,0,0,0);
    }
    return result;


}

int Simple_Tracker::contourOverlap(contour& ct_1, contour& ct_2){
    pair<Rect,Rect> intersec = rectIntersection(ct_1.pos_in_binary,ct_2.pos_in_binary);
    if(intersec.first.width==0 || intersec.second.height==0){
        return -1;
    }
    else{
        Mat roi_copy = cv::Mat::zeros(ct_1.binary.rows,ct_1.binary.cols,ct_1.binary.type());
        ct_2.binary(intersec.second).copyTo(roi_copy(intersec.first));
        Mat combined = ct_1.binary & roi_copy;
        return (int)((double)(cv::sum(combined)[0])/255.0);
    }
}
