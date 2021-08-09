/*
    ボールを床に落とす
*/

use nannou::prelude::*;
use rapier2d::prelude::*;
use crate::nalgebra::Isometry2;
use std::ops::Mul;

const WIDTH:  u32 = 960;
const HEIGHT: u32 = 540;
const SCALE:  f32 = 100.0;

const LEAF_W: f32 =  60.0;
const LEAF_H: f32 =   2.5;
const LEAF_LEN: f32 = 2.0;

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

  //  羽根
  //  くるくる
  let mut shape = vec![];
  for i in 0..LEAF_LEN as usize {
    shape.push((
      Isometry2::new(vector![0.0, 0.0], PI * 1.0 / LEAF_LEN * i as f32),
      SharedShape::cuboid(LEAF_W / SCALE, LEAF_H / SCALE)
    ));
  }
  let collider = ColliderBuilder::compound(shape).build();

  let rigid_body = RigidBodyBuilder::new_dynamic().lock_translations().build();
  let handle = rigid_body_set.insert(rigid_body);
  collider_set.insert_with_parent(collider, handle, &mut rigid_body_set);

  //  ボール
  let rigid_body = RigidBodyBuilder::new_dynamic()
    .translation(vector![1.0 / SCALE, 200.0 / SCALE])
    .build();
  let collider = ColliderBuilder::ball(5.0 / SCALE)
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

fn add_ball(model: &mut Model) {
  let rigid_body = RigidBodyBuilder::new_dynamic()
    .translation(vector![
      (random_f32() * LEAF_W - LEAF_W * 0.5) /SCALE, 
      HEIGHT as f32 * 0.5 / SCALE
    ])
    .build();
  let collider = ColliderBuilder::ball(5.0 / SCALE).restitution(0.7).build();
  let ball_body_handle = model.rigid_body_set.insert(rigid_body);
  model.collider_set.insert_with_parent(collider, ball_body_handle, &mut model.rigid_body_set);
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

  let mut hashes = vec!();
  for (h, b) in model.rigid_body_set.iter() {
    if b.translation().y < HEIGHT as f32 * -0.5 / SCALE {
      hashes.push(h); //  削除するもの
    }
  }

  //  remove
  for h in hashes {
    model.rigid_body_set.remove(
      h,
      &mut model.island_manager,
      &mut model.collider_set,
      &mut model.joint_set
    );

    add_ball(model);
  }
}

fn view(app: &App, model: &Model, frame: Frame) {
  let draw = app.draw();
  draw.background().color(INDIGO);

  //  ボールの表示
  for (h, b) in model.rigid_body_set.iter() {
    let p = pt2(b.translation().x, b.translation().y).mul(SCALE);
    if h.into_raw_parts().0 < 1 {
      // くるくる
      let r = b.rotation().angle();
      for i in 0..LEAF_LEN as usize {
        let rb = PI * 1.0 / LEAF_LEN * i as f32;
        draw.rect()
          .w_h(LEAF_W * 2.0, LEAF_H * 2.0)
          .xy(p)
          .rotate(r + rb)
          .color(hsv(rb, 1.0, 1.0));

        //  CENTER
        draw.rect()
          .w_h(LEAF_H * 2.0, LEAF_H * 2.0)
          .rotate(r)
          .color(INDIGO);
        }
    } else {
      draw.ellipse()
        .w_h(10.0, 10.0)
        .xy(p)
        .color(WHITE);
    }
  }

  draw.to_frame(app, &frame).unwrap();
}
