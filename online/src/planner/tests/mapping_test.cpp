#include <catch_ros/catch.hpp>
#include "planner/planner.hpp"

TEST_CASE("Tests is_valid function","[is_valid]"){
    
    using namespace planner;
    using arma::mat;
    using std::string;

    mat grid = { {0, 0, 0, 0, 1},
                 {0, 0, 0, 0, 1},
                 {0, 0, 0, 0, 1},
                 {0, 0, 0, 0, 1},
                 {0, 0, 0, 0, 1} };

    Map map(grid,2.5,2.5,0.5);
    Node q(1.5,1.5,nullptr);

    bool result1 = map.is_valid(&q,{1.5,1.5});
    bool result2 = map.is_valid(&q,{0,0});

    REQUIRE(result1==false);
    REQUIRE(result2==true);
    
}

TEST_CASE("Tests new_config function (checks new node placement)","[new_config]"){

    using namespace planner;
    using std::vector;

    mat grid = { {0, 0, 0, 0, 1},
                 {0, 0, 0, 0, 1},
                 {0, 0, 0, 0, 1},
                 {0, 0, 0, 0, 1},
                 {0, 0, 0, 0, 1} };

    Map map(grid,2.5,2.5,0.5);

    Node q(1,1,nullptr);
    Node q_near(2,2,nullptr);
    Node* q_new;

    map.new_config(&q,&q_near,q_new,{0.1,0.2});

    std::cout << q_new->x << std::endl;

    REQUIRE(is_equal(q_new->x,2-.134350288));
    REQUIRE(is_equal(q_new->y,2-.134350288));

}

TEST_CASE("Tests distance() function","[distance]"){

    using namespace planner;

    Node q1(1,1,nullptr);
    Node q2(4,5,nullptr);

    REQUIRE(is_equal(q1.distance(&q2),5));
}

TEST_CASE("Tests the rrt extend function","[extend]"){

    using namespace planner;
    using std::vector;
    using std::string;

    arma::mat test(500,500,arma::fill::zeros);
    Map map(test,25,25,0.05);

    Node t1(3,3,nullptr);
    Node t2(4,3,&t1);
    vector<Node*> tree = {&t1,&t2};
    Node q(7,6,nullptr);
    Node* q_new;
  
    string result1 = map.extend(tree,&q,q_new,{1,1});
    string result2 = map.extend(tree,&q,q_new,{1,1});

    REQUIRE(result1=="Advanced");
    REQUIRE(result2=="Reached");


}