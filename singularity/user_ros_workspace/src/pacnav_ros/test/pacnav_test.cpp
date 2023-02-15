#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch2_tester.h"
#include "pacnav_controller.h"

/* Three paths in 2d //{ */

TEST_CASE("Three paths in 2d", "[similarity_2d]") {

  std_msgs::Header test_header;
  test_header.frame_id = "test_frame";

  map<int, vector<geometry_msgs::PointStamped>> paths;

  paths[0].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(0, 0, 0)));
  paths[0].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(2, 0, 0)));

  paths[1].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(2, 2, 0)));
  paths[1].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(4, 4, 0)));

  paths[2].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(-2, -2, 0)));
  paths[2].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(-4, -4, 0)));

  REQUIRE(paths.size() == 3);
  for (int it = 0; it < paths.size(); it++) {
    REQUIRE(paths[it].size() == 2);
  }
  Eigen::Matrix3f des_mat;
  des_mat << 
    -1.0, 1.0/sqrt(2.0), -1.0/sqrt(2.0),
    1.0/sqrt(2.0), -1.0, -1.0,
    -1.0/sqrt(2.0), -1.0, -1.0;

  map<int, map<int, float>> similarity_mat = pacnav::getSimilarityMat(paths);
  for (int it = 0; it < paths.size(); it++) {

    for (int it2 = 0; it2 < paths.size(); it2++) {

      REQUIRE(similarity_mat[it][it2] == des_mat(it, it2));
    } 
  }
}

//}

/* Three long paths in 3d //{ */

TEST_CASE("Three long paths in 3d", "[similarity_3d]") {

  std_msgs::Header test_header;
  test_header.frame_id = "test_frame";
  map<int, vector<geometry_msgs::PointStamped>> paths;

  paths[0].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(0, 0, 0)));
  paths[0].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(1, 0, 0)));
  paths[0].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(2, 2, 0)));
  paths[0].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(2, 4, 0)));

  paths[1].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(1, 1, 1)));
  paths[1].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(2, 2, 2)));
  paths[1].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(3, 3, 3)));
  paths[1].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(4, 4, 4)));

  paths[2].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(-1, 1, 1)));
  paths[2].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(-2, 2, 2)));
  paths[2].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(-3, 3, 3)));
  paths[2].push_back(swm_r_utils::createPointStamped(test_header, swm_r_utils::createPoint(-4, 4, 4)));

  REQUIRE(paths.size() == 3);
  for (int it = 0; it < paths.size(); it++) {
    REQUIRE( paths[it].size() == 4 );
  }

  Eigen::Matrix3f des_mat;
  des_mat << 
    3.0 / (2 * sqrt(5.0)), (3.0 + 2 * sqrt(5.0)) / (3 * sqrt(15.0)), 1.0 / (3 * sqrt(15.0)),
    (3.0 + 2 * sqrt(5.0)) / (3 * sqrt(15.0)) , 1.0, 1.0 / 3.0,
    1.0 / (3 * sqrt(15.0)), 1.0 / 3.0, 1.0;

  map<int, map<int, float>> similarity_mat = pacnav::getSimilarityMat(paths);

  SECTION("3d path similarity check") {
    for (int it = 0; it < paths.size(); it++) {

      for (int it2 = 0; it2 < paths.size(); it2++) {

        REQUIRE(similarity_mat[it][it2] == des_mat(it, it2));
      } 
    }
  }

  SECTION("Cand convergence check") {

    geometry_msgs::Point target = paths[1].rbegin()->point;

    vector<bool> des_convg{true, false};

    for (int it = 0; it < paths.size(); it++) {

      if(it != 1) {
        REQUIRE(pacnav::isCandConverging(target, paths[it]) == des_convg[it]);
      }
    }

  }
}

//}

