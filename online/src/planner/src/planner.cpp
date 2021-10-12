#include "planner/planner.hpp"
#include <ctime>


namespace planner{

    using arma::mat;
    using std::string;
    using nav_msgs::OccupancyGrid;
    

    bool is_equal(const double &r, const double &l){
        return fabs(r - l) < 1e-5;
    }

    Path nav_path(const vector<pair<double,double>> &path, const std::string &frame_id){

        // initialize path and pose
        geometry_msgs::PoseStamped pose;
        nav_msgs::Path poses;
        
        poses.header.frame_id = frame_id;
        pose.header.frame_id = frame_id;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        // convert each waypoint in path to pose and add it to path
        for (auto point : path){
            pose.pose.position.x = point.first;
            pose.pose.position.y = point.second;

            poses.poses.push_back(pose);
        }

        return poses;
    }
    
    

    /********************
     * Node functions
    ********************/

    Node::Node(double x, double y, Node* parent) : x(x), y(y), parent(parent) {}

    double Node::distance(Node *q){
        return sqrt(pow(x - q->x,2) + pow(y - q->y,2));
    }

    vector<pair<double,double>> Node::get_path(){

        // initialize path vector with node position
        vector<pair<double,double>> path = {{x,y}};
        Node current = *this;

        // iterate back through linked parent nodes and append their position to path vector
        while (current.parent != nullptr){
            current = *current.parent;
            path.push_back({current.x,current.y});
        }

        return path;
    }

    /*********************
     * Map Functions
    ********************/

   OccupancyGrid Map::get_grid(string frame_id){

        OccupancyGrid out;
        out.header.frame_id = frame_id;
        out.info.resolution = dl;
        out.info.width = width/dl;
        out.info.height = height/dl;

        out.data.reserve(grid.n_elem);
        for (int x = 0; x < grid.n_cols; x++) {
            for (int y = 0; y < grid.n_rows; y++) {
                const unsigned char intensity = (unsigned)grid(x,y);
                out.data.push_back(intensity);

            }
        }


        return out;

    }

    Node Map::random_config(){


        double x = (double)rand()/RAND_MAX * width;
        double y = (double)rand()/RAND_MAX * height;

        return Node(x,y,nullptr);

    }

    bool Map::new_config(Node* q, Node* q_near, Node* &q_new, const vector<double> &TEB){
        
        // make q_new by propogating from q_near toward q
        double epsilon = *min_element(TEB.begin(),TEB.end()) * 1.9;
        double dist = q_near->distance(q);

        double dx = (q->x - q_near->x) * epsilon/dist;
        double dy = (q->y - q_near->y) * epsilon/dist;

        q_new = new Node(q_near->x + dx, q_near->y + dy, q_near);
        
        // check if q_new is valid

        return is_valid(q_new,TEB);

    }


    bool Map::is_valid(Node* q, const vector<double> &TEB){

        // get bounds of error checking
        double xlow = q->x - TEB[0];
        double xhigh = q->x + TEB[0];
        double ylow = q->y - TEB[1];
        double yhigh = q->y + TEB[1];


        // verify error bound is within map
        if (xlow < 0){
            xlow = 0;
        }
        if (ylow < 0){
            ylow = 0;
        }
        if (xhigh > width - dl){
            xhigh = width - dl;
        }
        if (yhigh > height - dl){
            yhigh = height - dl;
        }

        // convert bounds to indexes
        int ix_low = (int)round(xlow/dl);
        int ix_high = (int)round(xhigh/dl);
        int iy_low = (int)round(ylow/dl);
        int iy_high = (int)round(yhigh/dl);

        for (int x = ix_low; x <= ix_high; x++){
            if (grid(iy_low,x) > 0 or grid(iy_high,x) > 0){
                return false;
            }
        }

        for (int y = iy_low; y <= iy_high; y++){
            if (grid(y,ix_low) > 0 or grid(y,ix_high) > 0){
                return false;
            }
        }
        
        return true;
    }

    string Map::extend(vector<Node*> &tree, Node* q, Node* &q_new, const vector<double> &TEB){

        double epsilon = *min_element(TEB.begin(),TEB.end()) * 1.9;

        Node* q_near = nearest_neighbor(tree,q);
        
        if (new_config(q,q_near,q_new,TEB)){
            tree.push_back(q_new);

            if (q->distance(q_new) < epsilon){
                return "Reached";
            } else{
                return "Advanced";
            }

        }

        return "Trapped";
        
    }

    string Map::connect(vector<Node*> &tree, Node* q, const vector<double> &TEB){

        string S = "Advanced";

        while (S == "Advanced"){
            Node* q_new;
            S = extend(tree,q,q_new,TEB);
        }

        return S;

    }

    Node* Map::nearest_neighbor(vector<Node*> &tree, Node *q){

        double max_dist = INT_MAX;
        Node* nearest;

        for (auto node : tree){
            double dist = node->distance(q);
            if (dist < max_dist){
                nearest = node;
                max_dist = dist;
            }
        }

        return nearest;
    }

    Map::Map(mat grid, double w, double h, double dl) : width(w), height(h), dl(dl), grid(grid) {}

    Map::Map(string filepath, double w, double h, double dl) : width(w), height(h), dl(dl) {
        grid.load(filepath);
    }

    mat Map::get_grid(){
        return grid;
    }

    vector<pair<double,double>> Map::rrt_connect(pair<double,double> start, pair<double,double> end, vector<double> TEB, int K){

        using std::vector;

        // initialize trees
        Node start_node(start.first,start.second,nullptr);
        Node end_node(end.first,end.second,nullptr);

        vector<Node*> Ta = {&start_node};
        vector<Node*> Tb = {&end_node};

        for (int k = 0; k<K; k++){

            Node q = random_config();
            Node* q_new;

            if (extend(Ta,&q,q_new,TEB) != "Trapped"){
                
                if (connect(Tb,q_new,TEB) == "Reached"){
                    vector<pair<double,double>> path1 = Tb[Tb.size()-1]->get_path();
                    vector<pair<double,double>> path2 = q_new->get_path();
                    vector<pair<double,double>> path;

                    path.reserve(path1.size() + path2.size());
                    if (path1[path1.size()-1].first == start.first and path1[path1.size()-1].second == start.second){
                        std::reverse(path1.begin(),path1.end());
                        path.insert(path.end(),path1.begin(),path1.end());
                        path.insert(path.end(),path2.begin(),path2.end());
                    } else {
                        std::reverse(path2.begin(),path2.end());
                        path.insert(path.end(),path2.begin(),path2.end());
                        path.insert(path.end(),path1.begin(),path1.end());
                    }

                    return path;
                }
            }

            Ta.swap(Tb);

        }

        vector<pair<double,double>> failure;
        return failure;
    }

}
