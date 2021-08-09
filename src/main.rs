/*
    ボールを床に落とす
*/

use nannou::prelude::*;
use rapier2d::prelude::*;
use std::ops::Mul;

const WIDTH:  u32 = 960;
const HEIGHT: u32 = 540;
const SCALE:  f32 = 100.0;

fn main() {
  nannou::app(model)
    .update(update)
    .run();
}

struct Model {
  gravity: Vector<Real>,
  integration_parameters: IntegrationParameters,
  island_manager: IslandManager,
  broad_phase: BroadPhase,
  narrow_phase: NarrowPhase,
  joint_set: JointSet,
  ccd_solver: CCDSolver,
  rigid_body_set: RigidBodySet,
  collider_set: ColliderSet,
  physics_pipeline: PhysicsPipeline
}

fn model(app: &App) -> Model {
  app.new_window()
    .size(WIDTH, HEIGHT)
    .view(view)
    .build().unwrap();
  
  let mut rigid_body_set = RigidBodySet::new(); //  物理パイプラインで扱うことのできるリジッドボディのセット
  let mut collider_set = ColliderSet::new();    //  物理ワールドで扱うことのできるコライダ(衝突器)のセット

  //  床
  let collider = ColliderBuilder::cuboid(100.0 / SCALE, 1.0 / SCALE)
    .translation(vector![0.0, -200.0 / SCALE])
    .build();
  collider_set.insert(collider);

  //  ボール
  let rigid_body = RigidBodyBuilder::new_dynamic()
    .translation(vector![0.0, 200.0 / SCALE])
    .build();
  let collider = ColliderBuilder::ball(10.0 / SCALE)
    .restitution(0.7) //  反発係数
    .build();
  let ball_body_handle = rigid_body_set.insert(rigid_body);
  collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

  Model {
    gravity: vector![0.0, -9.81],
    integration_parameters: IntegrationParameters::default(),
    island_manager: IslandManager::new(),
    broad_phase: BroadPhase::new(),
    narrow_phase: NarrowPhase::new(),
    joint_set: JointSet::new(),
    ccd_solver: CCDSolver::new(),
    rigid_body_set: rigid_body_set,
    physics_pipeline: PhysicsPipeline::new(),
    collider_set: collider_set
  }
}

fn update(_app: &App, model: &mut Model, _update: Update) {
  model.physics_pipeline.step(
    &model.gravity,
    &model.integration_parameters,
    &mut model.island_manager,
    &mut model.broad_phase,
    &mut model.narrow_phase,
    &mut model.rigid_body_set,
    &mut model.collider_set,
    &mut model.joint_set,
    &mut model.ccd_solver,
    &(),
    &(),
  );
}

fn view(app: &App, model: &Model, frame: Frame) {
  let draw = app.draw();
  draw.background().color(INDIGO);

  //  床の表示
  draw.rect()
    .w_h(200.0, 2.0)
    .x_y(0.0, -200.0)
    .color(WHITE);

  //  ボールの表示
  for (_h, b) in model.rigid_body_set.iter() {
    let p = pt2(b.translation().x, b.translation().y).mul(SCALE);
    draw.ellipse()
      .w_h(20.0, 20.0)
      .xy(p)
      .color(WHITE);
  }

  draw.to_frame(app, &frame).unwrap();
}
