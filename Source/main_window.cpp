#include <QDir>
#include <QFileDialog>
#include <QDesktopServices>
#include <QTime>
#include <QElapsedTimer>


#include "main_window.h"
#include "ui_main_window.h"

Main_Window::Main_Window(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Main_Window),
    simple_tracker(&state),
    rjmcmc_tracker(&state)
{
    ui->setupUi(this);
    this->setObjectName("Main Window");
    this->setWindowTitle("Drosophila Tracker");

    //set standard values
    ui->lineEdit_N->setText(QString::number(state.sampling.N));
    ui->lineEdit_M->setText(QString::number(state.sampling.M));
    ui->lineEdit_B->setText(QString::number(100.0*(float)state.sampling.B/(float)state.sampling.N));
    ui->lineEdit_FPS->setText(QString::number(state.sampling.fps));
    ui->lineEdit_scaling->setText(QString::number(state.sampling.scaling_factor));
    ui->lineEdit_aThresh->setText(QString::number(state.sampling.a_threshold));

    //load io.conf if available
    ifstream config("io.conf");
    if(config.is_open()){
        string input_1, input_2, input_3;
        config >> input_1;
        state.io.coll_dir=QString::fromStdString(input_1);
        config >> input_2;
        state.io.output_dir=QString::fromStdString(input_2);
        config >> input_3;
        state.io.seq_dir=QString::fromStdString(input_3);

        config.close();

        QDir dir(state.io.coll_dir);
        if(!dir.exists()){
            state.io.coll_dir="";
        }
        else{
            ui->lineEdit_colls->setText(state.io.coll_dir);
        }

        QDir dir_out(state.io.output_dir);
        if(!dir_out.exists()){
            state.io.output_dir="";
        }
        else{
            ui->lineEdit_output->setText(state.io.output_dir);
        }

        QDir dir_seq(state.io.seq_dir);
        if(!dir_seq.exists()){
            state.io.seq_dir="";
        }
        else{
            ui->lineEdit_seq->setText(state.io.seq_dir);
        }

        STATUS("Config loaded.");
    }

    //connect logger and progressbar
    connect(this, &Main_Window::log, this, &Main_Window::setLogText);
    connect(&this->simple_tracker, &Simple_Tracker::log, this, &Main_Window::setLogText);
    connect(&this->simple_tracker, &Simple_Tracker::progress, [this](float value){ui->progressBar->setValue((int)(value*100.0));ui->progressBar->repaint();});

    //connect input/output buttons
    connect(ui->button_open_coll, &QPushButton::clicked, this, &Main_Window::openDir);
    connect(ui->button_open_out, &QPushButton::clicked, this, &Main_Window::openDir);
    connect(ui->button_open_seq, &QPushButton::clicked, this, &Main_Window::openDir);
    connect(ui->button_open_cur_seq, &QPushButton::clicked, this, &Main_Window::openDir);
    connect(ui->button_select_coll, &QPushButton::clicked, this, &Main_Window::selectDir);
    connect(ui->button_select_out, &QPushButton::clicked, this, &Main_Window::selectDir);
    connect(ui->button_select_seq, &QPushButton::clicked, this, &Main_Window::selectDir);

    //connect manual evaluation
    connect(ui->checkBox_manual_eval, &QCheckBox::clicked,[this](bool clicked){
        if(clicked){ui->groupBox_manual->show();}
        else{ui->groupBox_manual->hide();}
        this->state.eval.manual_evaluation=clicked;}
    );
    connect(ui->checkBox_gray_output, &QCheckBox::clicked,[this](bool clicked){state.eval.save_gray=clicked;});
    connect(ui->checkBox_binary_output, &QCheckBox::clicked,[this](bool clicked){state.eval.save_binary=clicked;});
    connect(ui->checkBox_copy_fails, &QCheckBox::clicked,[this](bool clicked){state.eval.copy_fails=clicked;});
    connect(ui->checkBox_show_output, &QCheckBox::clicked,[this](bool clicked){state.eval.show_output=clicked;});

    connect(ui->button_accept_right, &QPushButton::clicked, this, &Main_Window::manualEvaluation);
    connect(ui->button_accept_wrong, &QPushButton::clicked, this, &Main_Window::manualEvaluation);
    connect(ui->button_delete, &QPushButton::clicked, this, &Main_Window::manualEvaluation);
    connect(ui->button_restart, &QPushButton::clicked, this, &Main_Window::manualEvaluation);
    connect(ui->slider_frame, &QSlider::valueChanged,[this](int value){if((int)state.eval.current_sequence.size()>value){
            ui->image_left->setPixmap(state.eval.current_sequence[value]);
            ui->image_right->setPixmap(state.eval.current_sequence_binary[value]);}});

    //connect buttons
    connect(ui->button_apply,&QPushButton::clicked,this, &Main_Window::applyParameters);
    connect(ui->button_coll_extract,&QPushButton::clicked,&this->simple_tracker, &Simple_Tracker::track);
    connect(ui->button_statistics,&QPushButton::clicked,this, &Main_Window::showStatistics);
    connect(ui->button_coll_solve,&QPushButton::clicked,this,&Main_Window::collisionSolving);
    connect(ui->button_start_multiple,&QPushButton::clicked,this,&Main_Window::multipleSolvingTests);
}

Main_Window::~Main_Window()
{
    delete ui;
}

void Main_Window::showStatistics(){
    //check if input dir exists
    QDir dir(state.io.coll_dir);
    if(!dir.exists() || state.io.coll_dir.size()==0){
        ERROR("Collision dir does not exist");
        return;
    }
    //check if input dir contains subdirs
    QStringList subdirs = dir.entryList(QDir::NoDotAndDotDot|QDir::AllEntries);
    if(subdirs.size()==0){
        ERROR("Could not find sequences in collision dir "+state.io.seq_dir);
        return;
    }
    //sort subdirs
    QCollator col;
    col.setNumericMode(true);
    col.setCaseSensitivity(Qt::CaseInsensitive);
    std::sort(subdirs.begin(), subdirs.end(), [&](const QString& a, const QString& b) {return col.compare(a, b) < 0;});

    //count number of subdirs according to sequence lengths
    vector<int> dir_counter(102);
    float num_of_frames = 0.0;
    float num_of_seqs = 0.0;
    for(QString& subdir_name : subdirs){

        //check if subdir contains collisions
        QDir subdir(state.io.coll_dir+"/"+subdir_name);
        QStringList collision_dirs = subdir.entryList(QDir::NoDotAndDotDot|QDir::AllEntries);
        //sort collision dirs
        std::sort(collision_dirs.begin(), collision_dirs.end(), [&](const QString& a, const QString& b) {return col.compare(a, b) < 0;});

        for(QString& coll_dir_name : collision_dirs){
            num_of_seqs++;
            subdir.cd(coll_dir_name);
            QStringList files = subdir.entryList({"*.png","*.jpg","*.jpeg","*.tif","*.tiff"});
            int num_of_entries = files.size();
            num_of_frames+=num_of_entries;
            if(num_of_entries>=100){
                dir_counter[101]++;
            }
            else{
                dir_counter[num_of_entries]++;
            }
            subdir.cdUp();
        }
    }

    STATUS("Statistics:");
    vector<int> combined;
    for(int i=0; i<20; i++){
        combined.push_back(0);
        for(int j=0;j<5;j++){
            combined.back()+=dir_counter[5*i+j+1];
        }
        STATUS(QString::number(5*i+1)+" to "+QString::number(5*i+5)+": "+QString::number(combined.back()));
    }
    STATUS(">100 : "+QString::number(dir_counter.back()));

    STATUS("Average frames per sequence: "+QString::number(num_of_frames/num_of_seqs));
}

void Main_Window::manualEvaluation(){
    QObject* sender = QObject::sender();
    if(sender->objectName()=="button_accept_right"){
        //save result mid circle positions
        sample& current_sample = state.sampling.best_sample;
        ofstream file(state.eval.current_sequence_dir.toStdString()+"/final_position.txt");
        if(file.is_open()){
            for(size_t i=0; i<current_sample.targets.size(); i++){
                target_data& tgt = current_sample.targets[i];
                file<<1.0/state.sampling.scaling_factor*tgt.model[3].p.x<<" "<<1.0/state.sampling.scaling_factor*tgt.model[3].p.y<<"\n";
            }
        }
        else{
            ERROR("Unable to save final positions.");
        }
        state.eval.correct_ids+=state.eval.num_of_current_ids;
        state.eval.sum_ids+=state.eval.num_of_current_ids;
    }
    else if(sender->objectName()=="button_accept_wrong"){
        QDir dir(state.eval.current_result_sequence_dir);
        if(!dir.rename(dir.absolutePath(),state.eval.current_results_dir+"/FAIL_"+dir.dirName())){
            ERROR("Unable to rename sequence result dir.");
        }
        state.eval.wrong_ids+=state.eval.num_of_current_ids;
        state.eval.sum_ids+=state.eval.num_of_current_ids;
        if(state.eval.copy_fails){//copy current sequence dir to FAIL dir
            copyDir(state.eval.current_sequence_dir,state.io.coll_dir+"/FAILED");
        }
    }
    else if(sender->objectName()=="button_delete"){
        QDir dir_1(state.eval.current_result_sequence_dir);
        if(!dir_1.removeRecursively()){
            ERROR("Unable to delete sequence result dir.");
        }
        QDir dir_2(state.eval.current_sequence_dir);
        if(!dir_2.removeRecursively()){
            ERROR("Unable to delete sequence dir.");
        }
    }
    else if(sender->objectName()=="button_restart"){
        (*state.eval.current_idx)--;
        //delete headpoints and headfile
        state.heads.headpoints.clear();
        QFile file(state.eval.current_sequence_dir+"/heads.txt");
        if(!file.remove()){
            ERROR("Unable to remove head file.");
        }
        QDir dir(state.eval.current_result_sequence_dir);
        if(!dir.removeRecursively()){
            ERROR("Unable to delete sequence result dir.");
        }
    }

    state.eval.sequence_evaluated=true;
}

void Main_Window::applyParameters(){
    bool save = true;

    int N,M,B;
    float scaling, fps, a_threshold;

    string input = ui->lineEdit_N->text().toStdString();
    N = stoi(input);
    if(N<=0){
        ERROR("Non-positive N not allowed.");
        save=false;
    }
    else if(N>5000){
        WARN("Large sampling rates N are not efficient to compute.");
    }

    input = ui->lineEdit_M->text().toStdString();
    M = stoi(input);
    if(M<=0){
        ERROR("Non-positive M not allowed.");
        save=false;
    }
    else if(M>10){
        WARN("By setting M this high, a lot of the samples have to be computed in order to get N result samples.");
    }

    input = ui->lineEdit_B->text().toStdString();
    B = (int)((float)(N)*0.01*stof(input));
    if(B<=0){
        ERROR("Non-positive B not allowed.");
        save=false;
    }
    else if(B>N/2){
        WARN("By setting B this high, a lot of the samples are thrown away.");
    }

    input = ui->lineEdit_FPS->text().toStdString();
    fps = stof(input);
    if(fps<=0.0){
        ERROR("Non-positive fps not allowed.");
        save=false;
    }

    input = ui->lineEdit_scaling->text().toStdString();
    scaling = stof(input);
    if(scaling<=0.0){
        ERROR("Non-positive scaling not allowed.");
        save=false;
    }

    input = ui->lineEdit_aThresh->text().toStdString();
    a_threshold = stof(input);
    if(a_threshold<0.0 || a_threshold>1.0){
        ERROR("Acceptance rate threshold has to be in [0,1].");
        save=false;
    }

    if(save){
        state.sampling.N=N;
        state.sampling.B=B;
        state.sampling.M=M;
        state.sampling.fps=fps;
        state.sampling.scaling_factor=scaling;
        state.sampling.a_threshold=a_threshold;
        STATUS("Parameters saved.");
    }
    else{
        ERROR("Parameters not saved.");
    }

}

void Main_Window::multipleSolvingTests(){
    //ask user for parameter file
    QString file_name_q = QFileDialog::getOpenFileName(Q_NULLPTR, tr("Load Parameter File"),"./",tr("Parameter File (*.txt)"));
    ifstream file(file_name_q.toStdString());
    if(file.is_open()){
        int num_of_tests;
        file>>num_of_tests;

        string test_name;
        int N,M,B;
        float scaling, fps, a_threshold;
        QString output_dir_original = state.io.output_dir;

        for(int i=0; i<num_of_tests; i++){
            //read test name and parameters
            file>>test_name;
            file>>N;
            file>>B;
            file>>M;
            file>>fps;
            file>>scaling;
            file>>a_threshold;
            state.sampling.N=N;
            state.sampling.B=B;
            state.sampling.M=M;
            state.sampling.fps=fps;
            state.sampling.scaling_factor=scaling;
            state.sampling.a_threshold=a_threshold;
            STATUS("Parameters loaded.");

            //change output dir according to test name and create dir if does not exist
            state.io.output_dir=output_dir_original+"/"+QString::fromStdString(test_name);
            QDir out_dir(state.io.output_dir);
            if(!out_dir.cd(QString::fromStdString(test_name))){
                out_dir.cdUp();
                out_dir.mkdir(QString::fromStdString(test_name));
            }

            //start collision solving
            collisionSolving();
        }

        applyParameters();
        state.io.output_dir = output_dir_original;
    }
    else{
        ERROR("File could not be read.");
        return;
    }
}

void Main_Window::collisionSolving(){
    STATUS("Start collision solving.");
    state.eval.time_needed = 0.0;
    ui->progressBar->setValue(0);
    ui->progressBar->repaint();

    //check if input dir exists
    QDir dir(state.io.coll_dir);
    if(!dir.exists() || state.io.coll_dir.size()==0){
        ERROR("Collision dir does not exist");
        return;
    }

    //check if output dir exists
    QDir dir_out(state.io.output_dir);
    if(!dir_out.exists() || state.io.output_dir.size()==0){
        ERROR("Output dir does not exist");
        return;
    }

    //check if input dir contains subdirs
    QStringList subdirs = dir.entryList(QDir::NoDotAndDotDot|QDir::AllEntries);
    if(subdirs.size()==0){
        ERROR("Could not find sequences in collision dir "+state.io.seq_dir);
        return;
    }

    //sort subdirs
    QCollator col;
    col.setNumericMode(true);
    col.setCaseSensitivity(Qt::CaseInsensitive);
    std::sort(subdirs.begin(), subdirs.end(), [&](const QString& a, const QString& b) {return col.compare(a, b) < 0;});

    for(QString& subdir_name : subdirs){
        //reset evaluation statistics
        state.eval.correct_ids = 0.0;
        state.eval.wrong_ids = 0.0;
        state.eval.sum_ids = 0.0;
        state.eval.avg_spine = 0.0;
        state.eval.avg_area = 0.0;

        state.eval.current_results_dir=state.io.output_dir+"/"+subdir_name;
        //check if subdir contains collisions
        QDir subdir(state.io.coll_dir+"/"+subdir_name);
        QStringList collision_dirs = subdir.entryList(QDir::NoDotAndDotDot|QDir::AllEntries);
        //sort collision dirs
        std::sort(collision_dirs.begin(), collision_dirs.end(), [&](const QString& a, const QString& b) {return col.compare(a, b) < 0;});

        for(size_t idx=0; idx<(size_t)collision_dirs.size(); idx++){
            state.eval.timer.restart();

            QString& collision_dir_name = collision_dirs[idx];
            ui->label_seq_name->setText(collision_dir_name);
            ui->label_seq_name->repaint();

            //reset evaluation data
            state.eval.current_sequence.clear();
            state.eval.current_sequence_binary.clear();
            state.eval.current_sequence_dir = state.io.coll_dir+"/"+subdir_name+"/"+collision_dir_name;
            state.heads.headfile_path = state.eval.current_sequence_dir.toStdString()+"/heads.txt";
            state.eval.current_idx=&idx;
            state.eval.sequence_evaluated = false;

            vector<Point2f> initial_positions;
            vector<Point2f> final_positions;
            vector<size_t> larva_ids;

            if(state.eval.manual_evaluation){//delete old position files
                QFile file(state.eval.current_sequence_dir+"/initial_positions.txt");
                file.remove();
                QFile file_final(state.eval.current_sequence_dir+"/final_position.txt");
                file_final.remove();
            }
            else{
                //read positions if available
                ifstream file_initial(state.eval.current_sequence_dir.toStdString()+"/initial_positions.txt");
                if(file_initial.is_open()){
                    Point2f p;
                    while(file_initial>>p.x && file_initial>>p.y){
                        initial_positions.push_back(state.sampling.scaling_factor*p);
                    }
                }
                file_initial.close();
                ifstream file_final(state.eval.current_sequence_dir.toStdString()+"/final_position.txt");
                if(file_final.is_open()){
                    Point2f p;
                    while(file_final>>p.x && file_final>>p.y){
                        final_positions.push_back(state.sampling.scaling_factor*p);
                    }
                }
                file_final.close();
            }

            //visually deactivate buttons
            ui->button_accept_right->setStyleSheet("color: rgb(125,125,125)");
            ui->button_accept_wrong->setStyleSheet("color: rgb(125,125,125)");
            ui->button_delete->setStyleSheet("color: rgb(125,125,125)");
            ui->button_restart->setStyleSheet("color: rgb(125,125,125)");

            QDir current_dir(state.io.coll_dir+"/"+subdir_name+"/"+collision_dir_name);
            QStringList image_file_names = current_dir.entryList({"*.tiff","*.tif","*.png"});
            if(image_file_names.size()==0){//if no images available try next dir
                continue;
            }

            //sort filenames
            std::sort(image_file_names.begin(), image_file_names.end(), [&](const QString& a, const QString& b) {return col.compare(a, b) < 0;});

            QString output_dir_name = state.io.output_dir+"/"+subdir_name+"/"+collision_dir_name+"/";
            state.eval.current_result_sequence_dir=output_dir_name;
            if(!dir_out.cd(subdir_name)){
                dir_out.mkdir(subdir_name);
                dir_out.cd(subdir_name);
            }

            if(!dir_out.cd(collision_dir_name)){
                dir_out.mkdir(collision_dir_name);
            }
            else{
                dir_out.cd("..");
            }
            dir_out.cd("..");

            sample current_sample;
            Mat image;
            Mat image_binary;
            float min_area = 0.0;

            rjmcmc_tracker.init();

            state.eval.num_of_frames = (float)image_file_names.size();
            ui->progressBar_seq->setValue(0);
            ui->progressBar_seq->repaint();

            ui->slider_frame->setMaximum(image_file_names.size()-1);
            ui->slider_frame->setValue(0);
            ui->slider_frame->repaint();

            size_t last_target_number = 0;
            float threshold =0.0; //threshold for binary thresholding

            state.eval.time_needed += state.eval.timer.elapsed();

            for(int img_idx = 0; img_idx<image_file_names.size(); img_idx++){
                state.sampling.frame=img_idx;
                QString& image_name = image_file_names[img_idx];
                string canonical_image_file_name = current_dir.canonicalPath().toStdString() + "/" + image_name.toStdString();

                //read current frame
                image = imread(canonical_image_file_name, CV_LOAD_IMAGE_COLOR);
                //rescale input image
                cv::resize(image, image, cv::Size(), state.sampling.scaling_factor,state.sampling.scaling_factor);
                //set image in system state
                state.sampling.current_image = image;

                vector<vector<Point>> detected_contours;
                if(state.sampling.frame==0){
                    //init background
                    ct_extractor.initBackground(&image, &threshold);

                    //detect contours
                    detected_contours = ct_extractor.extractContours(&image, &image_binary, state.sampling.scaling_factor*100.0, state.sampling.scaling_factor*5000.0, threshold);

                    //tracking
                    current_sample = rjmcmc_tracker.track(&image_binary, detected_contours);

                    //calculate area for contour filtering
                    min_area = state.sampling.avg_spine_length*state.sampling.avg_spine_length/10.0;
                }
                else{
                    //detect contours
                    detected_contours = ct_extractor.extractContours(&image, &image_binary, min_area, min_area*100.0, threshold);

                    //track
                    current_sample=rjmcmc_tracker.track(&image_binary, detected_contours);
                }

                if(current_sample.targets.size()==0){
                    break;
                }

                if(current_sample.targets.size()>last_target_number){
                    if(state.eval.manual_evaluation){//save new targets mid positions
                        ofstream file;
                        file.open(state.eval.current_sequence_dir.toStdString()+"/initial_positions.txt", std::ofstream::out | std::ofstream::app);
                        if(file.is_open()){
                            for(size_t i=last_target_number; i<current_sample.targets.size(); i++){
                                target_data& tgt = current_sample.targets[i];
                                file<<1.0/state.sampling.scaling_factor*tgt.model[3].p.x<<" "<<1.0/state.sampling.scaling_factor*tgt.model[3].p.y<<"\n";
                            }
                        }
                        else{
                            ERROR("Unable to save final positions.");
                        }
                    }
                    else{//compare to saved initial positions if available and save nearest idx
                        for(size_t i=last_target_number; i<current_sample.targets.size(); i++){
                            target_data& tgt = current_sample.targets[i];
                            size_t nearest_saved_point = 0;
                            double best_dist = 10000000.0f;
                            for(size_t pos_idx=0; pos_idx<initial_positions.size(); pos_idx++){
                                Point2f& p = initial_positions[pos_idx];
                                double dist = Basic_Calc::metricEucl(tgt.model[3].p,p);
                                if(dist<best_dist){
                                    best_dist=dist;
                                    nearest_saved_point=pos_idx;
                                }
                            }
                            larva_ids.push_back(nearest_saved_point);
                        }
                    }
                    last_target_number = current_sample.targets.size();
                }
                //draw sample onto image and save to output dir
                drawTargets(&current_sample,&image,&image_binary,1.0,output_dir_name.toStdString(),image_name.toStdString(), img_idx);

                //show rate of accepted samples
                ui->label_current_rate->setText(QString::number(state.sampling.current_rate_of_accepted));
                ui->label_current_rate->repaint();
            }

            if(current_sample.targets.size()==0){//delete sequence if no larvae in initial frame
                log("No larvae in initial frame. Please press delete sequence button!", Consts::LOG_WARNING);
                //wait for button to be pushed
                if(!state.eval.manual_evaluation){
                    ui->groupBox_manual->show();
                }

                ui->button_delete->setStyleSheet("color: rgb(255, 0, 0)");

                while(!state.eval.sequence_evaluated){
                    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
                }
                if(!state.eval.manual_evaluation){
                    ui->groupBox_manual->hide();
                }
                ui->button_delete->setStyleSheet("color: rgb(125, 125, 125)");
            }
            else{
                state.eval.avg_spine += state.sampling.avg_spine_length/(float)collision_dirs.size();
                state.eval.avg_area += state.sampling.avg_larva_area/(float)collision_dirs.size();
                state.eval.num_of_current_ids = state.heads.headpoints.size();

                //solution evaluation
                if(state.eval.manual_evaluation){//manual evaluation
                    //activate buttons
                    ui->button_accept_right->setStyleSheet("color: rgb(0, 0, 0)");
                    ui->button_accept_wrong->setStyleSheet("color: rgb(0, 0, 0)");
                    ui->button_delete->setStyleSheet("color: rgb(0, 0, 0)");
                    ui->button_restart->setStyleSheet("color: rgb(0, 0, 0)");
                    //wait for buttons to be pushed
                    while(!state.eval.sequence_evaluated){
                        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
                    }
                }
                else{//automatic evaluation
                    state.eval.timer.restart();
                    if(final_positions.size()==current_sample.targets.size()){
                        bool success=true;
                        //check if target mid circles are near saved result positions
                        for(size_t tgt_idx=0; tgt_idx<current_sample.targets.size(); tgt_idx++){
                            target_data& tgt = current_sample.targets[tgt_idx];
                            size_t nearest = 0;
                            double best_dist = 10000000.0f;

                            for(size_t pos_idx=0; pos_idx<final_positions.size(); pos_idx++){
                                Point2f& p = final_positions[pos_idx];
                                double dist = Basic_Calc::metricEucl(tgt.model[3].p,p);
                                if(dist<best_dist){
                                    best_dist=dist;
                                    nearest=pos_idx;
                                }
                            }
                            if(nearest!=larva_ids[tgt_idx] || best_dist>1.5*tgt.model[3].r){//Fail
                                success=false;
                                break;
                            }
                        }
                        if(!success){
                            //rename result folder
                            QDir dir(state.eval.current_result_sequence_dir);
                            if(!dir.rename(dir.absolutePath(),state.eval.current_results_dir+"/FAIL_"+dir.dirName())){
                                ERROR("Unable to rename sequence result dir.");
                            }
                            //copy current sequence dir to FAIL dir
                            if(state.eval.copy_fails){
                                copyDir(state.eval.current_sequence_dir,state.io.coll_dir+"/FAILED");
                            }

                            state.eval.wrong_ids+=state.eval.num_of_current_ids;
                        }
                        else{
                            state.eval.correct_ids+=state.eval.num_of_current_ids;
                        }
                        state.eval.sum_ids+=state.eval.num_of_current_ids;
                    }
                    else{
                        //rename result folder
                        QDir dir(state.eval.current_result_sequence_dir);
                        if(!dir.rename(dir.absolutePath(),state.eval.current_results_dir+"/UNKNOWN_"+dir.dirName())){
                            ERROR("Unable to rename sequence result dir.");
                        }
                        state.eval.sum_ids+=state.eval.num_of_current_ids;
                    }
                    state.eval.time_needed+=state.eval.timer.elapsed();
                }

                QString msg = QString::number(state.eval.correct_ids)+" of "+QString::number(state.eval.sum_ids)+" Rate: "+QString::number(state.eval.correct_ids/state.eval.sum_ids);
                ui->label_statistics->setText(msg);
                ui->label_statistics->repaint();

                ui->progressBar->setValue((int)(100.0*(float)(idx+1)/(float)collision_dirs.size()));
                ui->progressBar->repaint();

                ui->label_time->setText(QString::number((float)(state.eval.time_needed)/1000.0)+"s");
                ui->label_time->repaint();
            }
        }

        STATUS("Results for dir "+subdir_name+": Correct identities: "+QString::number(state.eval.correct_ids)+" / "+QString::number(state.eval.sum_ids));
        //write results to file
        ofstream result(state.io.output_dir.toStdString()+"/results.txt", std::ofstream::out | std::ofstream::app);
        if(result.is_open()){
            result<<state.eval.correct_ids<<" / "<<state.eval.sum_ids<<"        "<<state.eval.correct_ids/state.eval.sum_ids<<"     "<<100.0*(1.0-state.eval.correct_ids/state.eval.sum_ids)<<"\n";
        }
    }
    ofstream result(state.io.output_dir.toStdString()+"/results.txt", std::ofstream::out | std::ofstream::app);
    if(result.is_open()){
        result<<"Time:      "<<(float)(state.eval.time_needed)/1000.0<<" s      "<<(float)(state.eval.time_needed)/60000.0<<" min\n";
        //write parameters
        result<<"N :"<<state.sampling.N<<"\n";
        result<<"B :"<<state.sampling.B<<"\n";
        result<<"M :"<<state.sampling.M<<"\n";
        result<<"fps :"<<state.sampling.fps<<"\n";
        result<<"scaling :"<<state.sampling.scaling_factor<<"\n";
        result<<"a_threshold :"<<state.sampling.a_threshold<<"\n";
        result<<"Avg. spine length :"<<state.eval.avg_spine<<"\n";
        result<<"Avg. larva area :"<<state.eval.avg_area<<"\n";
    }
    STATUS("Done!");
}

void Main_Window::selectDir(){
    //open file dialog
    QString dir_name_q = QFileDialog::getExistingDirectory(this,"Choose a directory","~",QFileDialog::ShowDirsOnly);

    string dir_name = dir_name_q.toStdString();
    if(dir_name.size()==0){
        STATUS("No directory chosen.");
        return;
    }

    QObject* sender = QObject::sender();
    if(sender->objectName()=="button_select_coll"){
        state.io.coll_dir=dir_name_q;
        ui->lineEdit_colls ->setText(dir_name_q);
        STATUS("Collision sequence dir set to"+dir_name_q);
    }
    else if(sender->objectName()=="button_select_out"){
        state.io.output_dir=dir_name_q;
        ui->lineEdit_output->setText(dir_name_q);
        STATUS("Output dir set to"+dir_name_q);
    }
    else if(sender->objectName()=="button_select_seq"){
        state.io.seq_dir=dir_name_q;
        ui->lineEdit_seq->setText(dir_name_q);
        STATUS("Input sequence dir set to"+dir_name_q);
    }

    //save dirs to io.conf file
    ofstream config("io.conf");
    if(config.is_open()){
        config<<state.io.coll_dir.toStdString()<<"\n";
        config<<state.io.output_dir.toStdString()<<"\n";
        config<<state.io.seq_dir.toStdString()<<"\n";
    }
    config.close();
}

void Main_Window::openDir(){
    QString dir_path;

    QObject* sender = QObject::sender();
    if(sender->objectName()=="button_open_coll"){
        dir_path = state.io.coll_dir;
    }
    else if(sender->objectName()=="button_open_out"){
        dir_path = state.io.output_dir;
    }
    else if(sender->objectName()=="button_open_seq"){
        dir_path = state.io.seq_dir;
    }
    else if(sender->objectName()=="button_open_cur_seq"){
        dir_path = state.eval.current_sequence_dir;
    }

    if(dir_path.size()==0){
        ERROR("No directory set.");
        return;
    }
    else{
        //check if dir exists
        QDir dir(dir_path);
        if(!dir.exists()){
            ERROR("Directory does not exist.");
            return;
        }
        else{
            //open dir in default file explorer
            QDesktopServices::openUrl(QUrl::fromLocalFile(dir_path));
        }
    }
}

void Main_Window::setLogText(QString msg, int type){
    QString new_log;
    if(type==Consts::LOG_ERROR){//set error text
        QObject* sender = QObject::sender();
        new_log+=QTime::currentTime().toString()+": "+sender->objectName()+": Error: ";
        ui->logger->setTextColor(QColor::fromRgb(255,0,0));
    }
    else if(type==Consts::LOG_NORMAL){//set status text
        new_log+=QTime::currentTime().toString()+": ";
        ui->logger->setTextColor(QColor::fromRgb(0,0,0));
    }
    else if(type==Consts::LOG_WARNING){//set warning text
        new_log+=QTime::currentTime().toString()+": Warning: ";
        ui->logger->setTextColor(QColor::fromRgb(255,128,0));
    }
    new_log+=msg;

    if(type==Consts::LOG_APPEND){//append msg to last logger entry
        ui->logger->insertPlainText(new_log);
    }
    else{
        ui->logger->append(new_log);
    }
    //keep the logger respnsive under heavy cpu load (TODO memo: move heavy computations to threads in order to keep the whole toolkit responsive)
    ui->logger->repaint();
}

void Main_Window::drawTargets(sample* sample, Mat* image, Mat* image_binary, float zoom, std::string const& output_dir_path, std::string const& image_name, size_t image_idx){

    Mat result, result_binary;

    if(state.eval.save_gray || state.eval.show_output){
        result = image->clone();
        cv::resize(result,result,Size(0,0),zoom,zoom);

        for(size_t j=0;j<sample->targets.size();j++){
            if(sample->is_active[j]){
                target_data& target = sample->targets[j];
                vector<circ>& current_model = target.model;
                for(int k=0;k<7;k++){
                    cv::circle(result,zoom*current_model[k].p,zoom*current_model[k].r,state.io.colors[j],1);
                }
                cv::circle(result, zoom*current_model[6].p, 2, cv::Scalar(0,10,220), CV_FILLED);
            }
        }
    }

    if(state.eval.save_binary || state.eval.show_output){
        result_binary = image_binary->clone();
        cv::cvtColor(result_binary,result_binary,CV_GRAY2BGR);
        cv::resize(result_binary,result_binary,Size(0,0),zoom,zoom);

        for(size_t j=0;j<sample->targets.size();j++){
            if(sample->is_active[j]){
                target_data& target = sample->targets[j];
                vector<circ>& current_model = target.model;
                for(int k=0;k<7;k++){
                    cv::circle(result_binary,zoom*current_model[k].p,zoom*current_model[k].r,state.io.colors[j],1);
                }
                cv::circle(result_binary, zoom*current_model[6].p, 2, cv::Scalar(0,10,220), CV_FILLED);
            }
        }
    }

    if(state.eval.show_output){
        state.eval.current_sequence.push_back(QPixmap::fromImage(QImage((unsigned char*) result.data, result.cols, result.rows, result.step, QImage::Format_RGB888)));
        state.eval.current_sequence_binary.push_back(QPixmap::fromImage(QImage((unsigned char*) result_binary.data, result_binary.cols, result_binary.rows, result_binary.step, QImage::Format_RGB888)));
        ui->image_left->setPixmap(state.eval.current_sequence.back());
        ui->image_left->repaint();
        ui->image_right->setPixmap(state.eval.current_sequence_binary.back());
        ui->image_right->repaint();

        if(image_idx!=0){
            ui->slider_frame->setValue(ui->slider_frame->value()+1);
        }
        ui->slider_frame->repaint();
    }

    if(state.eval.save_gray){
        imwrite(output_dir_path+image_name, result);
    }
    if(state.eval.save_binary){
        imwrite(output_dir_path+"bin_"+image_name, result_binary);
    }


    ui->progressBar_seq->setValue((int)(100.0*((float)state.sampling.frame+1.0)/state.eval.num_of_frames));
    ui->progressBar_seq->repaint();
}

void Main_Window::copyDir(QString source, QString target){
    QDir source_dir(source);
    QDir target_dir(target);
    QString target_folder_name = target_dir.dirName();
    if(!source_dir.exists()){
        ERROR("Source sequence dir does not exist.");
        return;
    }
    if(!target_dir.exists()){
        target_dir.cd(state.io.coll_dir);
        if(!target_dir.mkdir(target_folder_name) || !target_dir.cd(target_folder_name)){
            ERROR("Target dir cannot be created.");
            return;
        }
    }
    if(!target_dir.mkdir(source_dir.dirName()) || !target_dir.cd(source_dir.dirName())){
        ERROR("Target dir cannot be created.");
        return;
    }
    //copy images
    QStringList files = source_dir.entryList(QDir::NoDotAndDotDot|QDir::AllEntries);
    for(QString& file_name : files){
        if(!QFile::copy(source+"/"+file_name,target_dir.absolutePath()+"/"+file_name)){
            ERROR("File cannot be copied.");
            return;
        }
    }
}
