#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "drape/core/collision_field.hpp"
#include "drape/core/fabric.hpp"
#include "drape/core/project.hpp"
#include "drape/core/sim_mesh.hpp"

namespace drape::sim {

using GarmentId = Id;

enum class SimPhase { Assembling, Settling };  // phase after build(): Settling

struct SimSettings {
  Vec3d gravity{0, -kGravity, 0};
  double collisionOffset = 0.003;       // m, added to half the fabric thickness
  double assemblyClosingSpeed = 0.25;   // m/s, seam rest-length shrink rate while Assembling
  double assemblyExtraDamping = 10.0;   // 1/s, added while Assembling
  double dampingScale = 5.0;            // 1/s per unit of fabric damping
  double strainLimitStiffnessScale = 100.0;  // strain-limit energy stiffness / stretch stiffness
  double seamStiffness = 1e4;           // N/m, seam spring stiffness
  double frictionSlipSpeed = 3e-4;      // m/s, slip speed at which body friction reaches mu * normal force
  int maxCgIterations = 50;             // PCG iterations per Newton step (inexact Newton)
  double cgTolerance = 1e-2;            // relative residual
};

struct SimGarment {
  GarmentId id;
  SimMesh mesh;
  std::vector<Vec3d> initialPositions;     // one per mesh vertex
  std::vector<FabricPhysical> pieceFabric;  // one per pattern piece index
};

struct Pin {
  std::uint32_t particle = 0;  // global particle index
  Vec3d position{0, 0, 0};
};

struct ExternalForce {
  std::uint32_t particle = 0;  // global particle index
  Vec3d force{0, 0, 0};        // newtons
};

struct SimScene {
  std::vector<SimGarment> garments;
  const CollisionField* body = nullptr;
  std::vector<Pin> pins;
  std::vector<ExternalForce> forces;
  SimSettings settings;
};

struct StepInput {
  double dt = 1.0 / 60.0;
  int substeps = 2;
  int iterations = 2;  // Newton iterations per substep
};

struct SimStats {
  bool hasNonFinite = false;
  double kineticEnergy = 0;
  double rmsSpeed = 0;
  double maxSeamGap = 0;
  double maxBodyPenetration = 0;
};

struct ParticleRange {
  std::uint32_t offset = 0;
  std::uint32_t count = 0;
};

struct Snapshot {
  std::vector<Vec3f> positions, velocities;
  std::vector<SimPhase> phases;
  std::vector<double> assemblyTime;
  std::vector<float> seamStartGap;
  std::vector<double> contactForce;  // per particle: last body normal force, which the next substep's friction uses
};

// Cloth solver interface (spec 5.2). Particles are the garments' vertices in scene order.
class Solver {
 public:
  virtual ~Solver() = default;
  virtual void build(const SimScene& scene) = 0;
  virtual void updateFabric(const GarmentId& g, const std::vector<FabricPhysical>& pieceFabric) = 0;
  virtual void setPhase(const GarmentId& g, SimPhase phase) = 0;
  virtual void step(const StepInput& in) = 0;
  virtual void readPositions(std::span<Vec3f> out) const = 0;  // out.size() == particleCount()
  virtual SimStats stats() const = 0;                          // valid right after build()
  virtual Snapshot snapshot() const = 0;
  virtual void restore(const Snapshot& s) = 0;
  virtual ParticleRange range(const GarmentId& g) const = 0;  // throws std::out_of_range for unknown ids
  virtual std::size_t particleCount() const = 0;
};

int substepsFor(SimQuality q);   // Draft 1, Standard 1, Fine 2
int iterationsFor(SimQuality q);  // Newton iterations: Draft 2, Standard 2, Fine 2

}  // namespace drape::sim
