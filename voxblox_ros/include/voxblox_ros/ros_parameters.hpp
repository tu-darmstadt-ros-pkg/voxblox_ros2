#ifndef VOXBLOX_ROS_ROS_PARAMS_H_
#define VOXBLOX_ROS_ROS_PARAMS_H_

#include <rclcpp/rclcpp.hpp>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <minkindr_conversions/kindr_xml.h>

namespace voxblox {

inline TsdfMap::Config getTsdfMapConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  TsdfMap::Config tsdf_config;

  /**
   * Workaround for OS X on mac mini not having specializations for float
   * for some reason.
   */
  double voxel_size = tsdf_config.tsdf_voxel_size;
  int voxels_per_side = tsdf_config.tsdf_voxels_per_side;

  voxel_size = node->declare_parameter<double>("tsdf_voxel_size", voxel_size);
  voxels_per_side =
      node->declare_parameter<int>("tsdf_voxels_per_side", voxels_per_side);

  if (!isPowerOfTwo(voxels_per_side)) {
    RCLCPP_ERROR(node->get_logger(),
                 "voxels_per_side must be a power of 2, setting to default "
                 "value");
    voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  }

  tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  tsdf_config.tsdf_voxels_per_side = voxels_per_side;

  return tsdf_config;
}

inline ICP::Config getICPConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  ICP::Config icp_config;

  icp_config.min_match_ratio = node->declare_parameter<double>(
      "icp_min_match_ratio", icp_config.min_match_ratio);
  icp_config.subsample_keep_ratio = node->declare_parameter<double>(
      "icp_subsample_keep_ratio", icp_config.subsample_keep_ratio);
  icp_config.mini_batch_size = node->declare_parameter<int>(
      "icp_mini_batch_size", icp_config.mini_batch_size);
  icp_config.refine_roll_pitch = node->declare_parameter<bool>(
      "icp_refine_roll_pitch", icp_config.refine_roll_pitch);
  icp_config.inital_translation_weighting =
      node->declare_parameter<double>("icp_inital_translation_weighting",
                                      icp_config.inital_translation_weighting);
  icp_config.inital_rotation_weighting =
      node->declare_parameter<double>("icp_inital_rotation_weighting",
                                      icp_config.inital_rotation_weighting);

  return icp_config;
}

inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  TsdfIntegratorBase::Config integrator_config;

  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);
  integrator_config.default_truncation_distance =
      tsdf_config.tsdf_voxel_size * 4;

  double truncation_distance = integrator_config.default_truncation_distance;
  double max_weight = integrator_config.max_weight;

  integrator_config.voxel_carving_enabled =
      node->declare_parameter<bool>("voxel_carving_enabled",
                                    integrator_config.voxel_carving_enabled);

  truncation_distance = node->declare_parameter<double>(
      "truncation_distance", truncation_distance);

  integrator_config.max_ray_length_m = node->declare_parameter<double>(
      "max_ray_length_m", integrator_config.max_ray_length_m);
  integrator_config.min_ray_length_m = node->declare_parameter<double>(
      "min_ray_length_m", integrator_config.min_ray_length_m);

  max_weight =
      node->declare_parameter<double>("max_weight", max_weight);

  integrator_config.use_const_weight = node->declare_parameter<bool>(
      "use_const_weight", integrator_config.use_const_weight);
  integrator_config.use_weight_dropoff = node->declare_parameter<bool>(
      "use_weight_dropoff", integrator_config.use_weight_dropoff);
  integrator_config.allow_clear = node->declare_parameter<bool>(
      "allow_clear", integrator_config.allow_clear);
  integrator_config.start_voxel_subsampling_factor =
      node->declare_parameter<int>(
          "start_voxel_subsampling_factor",
          integrator_config.start_voxel_subsampling_factor);
  integrator_config.max_consecutive_ray_collisions =
      node->declare_parameter<int>(
          "max_consecutive_ray_collisions",
          integrator_config.max_consecutive_ray_collisions);
  integrator_config.clear_checks_every_n_frames =
      node->declare_parameter<int>(
          "clear_checks_every_n_frames",
          integrator_config.clear_checks_every_n_frames);
  integrator_config.max_integration_time_s =
      node->declare_parameter<double>("max_integration_time_s",
                                      integrator_config.max_integration_time_s);
  integrator_config.enable_anti_grazing = node->declare_parameter<bool>(
      "anti_grazing", integrator_config.enable_anti_grazing);
  integrator_config.use_sparsity_compensation_factor =
      node->declare_parameter<bool>(
          "use_sparsity_compensation_factor",
          integrator_config.use_sparsity_compensation_factor);
  integrator_config.sparsity_compensation_factor =
      node->declare_parameter<double>(
          "sparsity_compensation_factor",
          integrator_config.sparsity_compensation_factor);
  integrator_config.integration_order_mode =
      node->declare_parameter<std::string>("integration_order_mode",
                                   integrator_config.integration_order_mode);

  integrator_config.default_truncation_distance =
      static_cast<float>(truncation_distance);
  integrator_config.max_weight = static_cast<float>(max_weight);

  return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  EsdfMap::Config esdf_config;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(node);
  esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
  esdf_config.esdf_voxels_per_side = tsdf_config.tsdf_voxels_per_side;

  return esdf_config;
}

inline EsdfIntegrator::Config getEsdfIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  EsdfIntegrator::Config esdf_integrator_config;

  TsdfIntegratorBase::Config tsdf_integrator_config =
      getTsdfIntegratorConfigFromRosParam(node);

  esdf_integrator_config.min_distance_m =
      tsdf_integrator_config.default_truncation_distance / 2.0;

  esdf_integrator_config.full_euclidean_distance =
      node->declare_parameter<bool>(
          "esdf_euclidean_distance",
          esdf_integrator_config.full_euclidean_distance);
  esdf_integrator_config.max_distance_m = node->declare_parameter<double>(
      "esdf_max_distance_m", esdf_integrator_config.max_distance_m);
  esdf_integrator_config.min_distance_m = node->declare_parameter<double>(
      "esdf_min_distance_m", esdf_integrator_config.min_distance_m);
  esdf_integrator_config.default_distance_m =
      node->declare_parameter<double>("esdf_default_distance_m",
                                      esdf_integrator_config.default_distance_m);
  esdf_integrator_config.min_diff_m = node->declare_parameter<double>(
      "esdf_min_diff_m", esdf_integrator_config.min_diff_m);
  esdf_integrator_config.clear_sphere_radius =
      node->declare_parameter<double>(
          "clear_sphere_radius", esdf_integrator_config.clear_sphere_radius);
  esdf_integrator_config.occupied_sphere_radius =
      node->declare_parameter<double>(
          "occupied_sphere_radius",
          esdf_integrator_config.occupied_sphere_radius);
  esdf_integrator_config.add_occupied_crust =
      node->declare_parameter<bool>(
          "esdf_add_occupied_crust",
          esdf_integrator_config.add_occupied_crust);

  if (esdf_integrator_config.default_distance_m <
      esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m =
        esdf_integrator_config.max_distance_m;
  }

  return esdf_integrator_config;
}

inline MeshIntegratorConfig getMeshIntegratorConfigFromRosParam(
    const rclcpp::Node::SharedPtr& node) {
  MeshIntegratorConfig mesh_integrator_config;

  mesh_integrator_config.min_weight =
      node->declare_parameter<double>("mesh_min_weight",
                                      mesh_integrator_config.min_weight);
  mesh_integrator_config.use_color =
      node->declare_parameter<bool>("mesh_use_color",
                                    mesh_integrator_config.use_color);

  return mesh_integrator_config;
}

inline std::vector<std::vector<double>> convert_to_matrix(
    const std::string& multilineString) {
  std::vector<std::vector<double>> result;

  std::istringstream iss(multilineString);
  std::string line;
  while (std::getline(iss, line)) {
    std::istringstream lineStream(line);
    std::vector<double> row;
    std::string valueStr;
    while (std::getline(lineStream, valueStr, ',')) {
      double value = std::stod(valueStr);
      row.push_back(value);
    }
    result.push_back(row);
  }

  return result;
}

inline voxblox::Transformation get_parameter_as_transformation(
    rclcpp::Node::SharedPtr node, std::string parameter_name) {
  voxblox::Transformation transformation;
  auto matrix_string = node->declare_parameter(parameter_name, "");
  if (!matrix_string.empty()) {
    std::vector<std::vector<double>> matrix =
        convert_to_matrix(matrix_string);
    kindr::minimal::vectorOfVectorsToKindr(matrix, &transformation);
  }
  return transformation;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ROS_PARAMS_H_
