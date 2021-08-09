use nannou::prelude::*;
use nannou::image::GenericImageView;
use rapier2d::prelude::*;
use crate::nalgebra::Isometry2;
use std::ops::Mul;
use std::ops::Add;

const WIDTH:  f32 = 960.0;
const HEIGHT: f32 = 540.0;
const HH: f32 = HEIGHT * 0.5;
const SCALE: f32 = 100.0;

const BALL_R: f32 =    1.8;
const BALL_COUNT: usize = 200;

const OBJ_SCALE: f32 = 0.5;
const PIX: f32 = 5.0;

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
  physics_pipeline: PhysicsPipeline,
  freeze: bool,
  points: Vec<Vec<Point2>>,
  image_len: usize
}

fn model(app: &App) -> Model {
  app.set_loop_mode(LoopMode::loop_once());
  app.new_window()
    .size(WIDTH as u32, HEIGHT as u32)
    .key_released(key_released)
    .view(view)
    .build().unwrap();

  let mut rigid_body_set = RigidBodySet::new(); //  物理パイプラインで扱うことのできるリジッドボディのセット
  let mut collider_set = ColliderSet::new();    //  物理ワールドで扱うことのできるコライダ(衝突器)のセット

  //  画像の読み込み
  let img = vec![
    nannou::image::open("src/example/images/ん.png").unwrap(),
    nannou::image::open("src/example/images/ろ.png").unwrap(),
    nannou::image::open("src/example/images/め.png").unwrap(),
  ];
  let (width, height) = img[0].dimensions();
  let l = width as f32 / PIX;

  let base = vec![-180.0, 0.0, 180.0];
  let mut pts = vec![];

  for i in 0..img.len() {
    let mut v = vec![vec![0; l as usize]; l as usize];
    let mut shape = vec![];
    let mut pt = vec![];

    for y in 0..height {
      for x in 0..width {
        if img[i].get_pixel(x, y)[0] > 250 {
          let px = (x / PIX as u32) as usize;
          let py = l as usize - (y / PIX as u32) as usize;

          if v[px][py] == 0 {
            let mut p = pt2(
              px as f32 * PIX - width as f32 * 0.5,
              py as f32 * PIX - height as f32 * 0.5
            ).mul(OBJ_SCALE);         
            p.y += base[i];
            pt.push(p);
            shape.push((
              Isometry2::new(vector![p.x / SCALE, p.y / SCALE], 0.0),
              SharedShape::cuboid(PIX * OBJ_SCALE * 0.5 / SCALE, PIX * OBJ_SCALE * 0.5 / SCALE)
            ));
            v[px][py] = 1;
          }
        }
      }
    }
    
    pts.push(pt);

    let rigid_body = RigidBodyBuilder::new_dynamic()
          .lock_translations().build();
    let body_handle = rigid_body_set.insert(rigid_body);
    let collider = ColliderBuilder::compound(shape).build();
    collider_set.insert_with_parent(collider, body_handle, &mut rigid_body_set);
  }

  //  balls
  for _i in 0..BALL_COUNT {
    let rigid_body = RigidBodyBuilder::new_dynamic()  //  体が外力や接触の影響を受けていることを示します。
          .translation(vector![ball_x_pos(), HH / SCALE + random_f32()])
          .build();
    let collider = ColliderBuilder::ball(BALL_R / SCALE)
          .build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);
  }

  println!("'F'キーで一時停止/解除");

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
    collider_set: collider_set,
    freeze: true,
    points: pts,
    image_len: img.len()
  }
}

fn key_released(app: &App, model: &mut Model, key: Key) {
  match key {
    Key::F => {
      model.freeze = !model.freeze;
      if model.freeze {
          app.set_loop_mode(LoopMode::loop_once());
      } else {
          app.set_loop_mode(LoopMode::RefreshSync);
      }
    }

    _ => (),
  }
}

fn ball_x_pos() -> f32 {
  (random_f32() * 100.0 - 50.0) /SCALE
}

fn add_ball(model: &mut Model) {
  let rigid_body = RigidBodyBuilder::new_dynamic()
    .translation(vector![ball_x_pos(), HH / SCALE])
    .build();
  let collider = ColliderBuilder::ball(BALL_R / SCALE).restitution(0.7).build();
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
    if h.into_raw_parts().0 > 0 {
      if b.translation().y < -HH / SCALE {
        //  削除するもの
        hashes.push(h);
      }
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
  
  for (h, b) in model.rigid_body_set.iter() {
    let p = pt2(b.translation().x, b.translation().y).mul(SCALE);

    if h.into_raw_parts().0 < model.image_len as u32 {
      let r = b.rotation().angle();
      for pt in model.points[h.into_raw_parts().0 as usize].iter() {
        let pos = pt2(
          pt.x * r.cos() - pt.y * r.sin(), 
          pt.x * r.sin() + pt.y * r.cos()
        ); 
        draw.rect()
          .w_h(PIX * OBJ_SCALE, PIX * OBJ_SCALE)
          .xy(p.add(pos))
          .rotate(r)
          .color(WHITE);
      }
    } else {
      draw.ellipse()
        .w_h(BALL_R * 2.0, BALL_R * 2.0)
        .xy(p)
        .color(WHITE);
    }
  }
  
  draw.to_frame(app, &frame).unwrap();
}
