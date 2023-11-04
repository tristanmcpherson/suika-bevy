use bevy::prelude::*;
use bevy::render::RenderPlugin;
use bevy::render::camera::ScalingMode;
use bevy::render::settings::{WgpuSettings, WgpuFeatures};
use bevy::window::{PrimaryWindow, WindowMode, WindowResolution};

use bevy_xpbd_2d::math::Vector;
use bevy_xpbd_2d::prelude::*;
use bevy_hanabi::prelude::*;

use std::cmp;
use std::collections::HashMap;
use std::string::ToString;
use std::time::Duration;
use strum::EnumCount;
use strum_macros;
use strum_macros::{Display, EnumCount, EnumIter, FromRepr};

use once_cell::sync::Lazy;

use rand::prelude::*;

// #[derive(Component)]
// struct Position(Vec2);

#[derive(Component)]
struct Player;

#[derive(Component)]
struct Name(String);

#[derive(Component)]
struct Controllable;

#[derive(Component)]
struct Spawner;

#[derive(Component)]
struct Holding(Entity);

#[derive(Component)]
struct Spawning;

#[derive(Component)]
struct Following;

#[derive(Component)]
struct Fruit(FruitType);

#[derive(Component)]
struct Combining(Entity, Position, Duration);

#[derive(Component)]
struct Growing(Duration, FruitType);

//
//#[derive()]
//struct Resizing;

#[derive(Resource)]
struct LogTimer(Timer);

#[derive(Resource)]
struct SpawnTimer(Timer);

#[derive(Component)]
struct ScoreText;

#[derive(Resource)]
struct Score(i64);

#[derive(Bundle)]
struct FruitBundle {
    sprite: SpriteBundle,
    fruit: Fruit,
    collider: Collider,
    collision_layers: CollisionLayers,
    restitution: Restitution
}

// Window config
pub const TITLE: &str = "SUIKA";
pub const WINDOW_HEIGHT: f32 = 754.0;
pub const WINDOW_WIDTH: f32 = 430.0;

// Monitor information
pub const MONITOR_HEIGHT: i32 = 1080;
pub const MONITOR_WIDTH: i32 = 1920;

fn main() {
    // let mut wgpu_settings = WgpuSettings::default();
    // wgpu_settings
    //     .features
    //     .set(WgpuFeatures::VERTEX_WRITABLE_STORAGE, true);

    App::new()

        .add_plugins(PhysicsPlugins::default())
        //.add_plugins(PhysicsDebugPlugin::default())
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                resolution: WindowResolution::new(WINDOW_WIDTH, WINDOW_HEIGHT),
                title: TITLE.to_string(),
                position: WindowPosition::At(IVec2::new(MONITOR_WIDTH / 4, MONITOR_HEIGHT / 4)),
                resizable: false,
                resize_constraints: WindowResizeConstraints {
                    min_width: WINDOW_WIDTH,
                    max_width: WINDOW_WIDTH,
                    min_height: WINDOW_HEIGHT,
                    max_height: WINDOW_HEIGHT,
                },
                mode: WindowMode::Windowed,
                ..default()
            }),
            ..default()
        }))
        // .set(RenderPlugin { wgpu_settings }))
        .add_plugins(HanabiPlugin)
        .insert_resource(SpawnTimer(Timer::from_seconds(0.25, TimerMode::Repeating)))
        .insert_resource(LogTimer(Timer::from_seconds(0.25, TimerMode::Repeating)))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .insert_resource(FruitPrefabs::new())
        .insert_resource(Score(0))
        .add_systems(Startup, setup)
        .add_systems(PostProcessCollisions, collision)
        .add_systems(Update, (input, spawn_timer, spawner, combining, grow, text_update_system))
        .run();
}

#[derive(Display, Debug, EnumIter, PartialEq, Eq, Hash, FromRepr, EnumCount, Clone, Copy)]
#[repr(u8)]
enum FruitType {
    Cherry = 1,
    Strawberry = 2,
    Grape = 3,
    Dekopon = 4,
    Orange = 5,
    Apple = 6,
    Pear = 7,
    Peach = 8,
    Pineapple = 9,
    Melon = 10,
    Watermelon = 11
}

#[derive(Debug)]
struct FruitPrefab {
    size: f32,
    points: i32,
}

// Create a struct to hold the HashMap and make it pub
#[derive(Resource)]
#[derive(Debug)]
struct FruitPrefabs {
    pub map: HashMap<FruitType, FruitPrefab>,
}

impl FruitPrefabs {
    pub fn new() -> Self {
        let mut map = HashMap::new();

        map.insert(
            FruitType::Cherry,
            FruitPrefab {
                size: 32.0,
                points: 2,
            },
        );
        map.insert(
            FruitType::Strawberry,
            FruitPrefab {
                size: 47.0,
                points: 4,
            },
        );
        map.insert(
            FruitType::Grape,
            FruitPrefab {
                size: 64.0,
                points: 6,
            },
        );
        map.insert(
            FruitType::Dekopon,
            FruitPrefab {
                size: 71.0,
                points: 8,
            },
        );
        map.insert(
            FruitType::Orange,
            FruitPrefab {
                size: 92.0,
                points: 10,
            },
        );
        map.insert(
            FruitType::Apple,
            FruitPrefab {
                size: 109.0,
                points: 12,
            },
        );
        map.insert(
            FruitType::Pear,
            FruitPrefab {
                size: 118.0,
                points: 14,
            },
        );

        map.insert(
            FruitType::Peach,
            FruitPrefab {
                size: 258.0 / 2.0,
                points: 16,
            },
        );

        map.insert(
            FruitType::Pineapple,
            FruitPrefab {
                size: 308.0 / 2.0,
                points: 18,
            },
        );

        map.insert(
            FruitType::Melon,
            FruitPrefab {
                size: 308.0 / 2.0,
                points: 20,
            },
        );

        map.insert(
            FruitType::Watermelon,
            FruitPrefab {
                size: 408.0 / 2.0,
                points: 22,
            },
        );
        FruitPrefabs { map }
    }
}


// lerp between two positions
fn lerp(a: f32, b: f32, t: f32) -> f32 {
    return a + (b - a) * t.min(1.0);
}

fn text_update_system(
    score: Res<Score>,
    mut query: Query<&mut Text, With<ScoreText>>,
) {
    for mut text in &mut query {
        // Update the value of the second section
        let score_value = score.0;
        text.sections[1].value = format!("{}", score_value);
    }
}

fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
    let mut camera = Camera2dBundle::default();
    // camera.projection.scale = 1.0;
    // camera.projection.scaling_mode = ScalingMode::FixedVertical(1.);

    commands.spawn(camera);
    commands.spawn((
        Player,
        Spawner,
        Controllable, //, Transform::from_xyz(0.0,0.0,0.0), LinearVelocity(Vec2{x: 0.0, y:0.0})
        Position(Vec2 {
            x: 0.0,
            y: (WINDOW_HEIGHT / 2.0) - 50.0,
        }),
    ));

    // Spawn blue platform that belongs on the blue layer and collides with blue
    spawn_wall(
        &mut commands,
        Vec3::new(0.0, -WINDOW_HEIGHT / 2.0 - 250.0 + 5.0, 0.0),
        Vec2::new(WINDOW_WIDTH, 500.0),
    );

    spawn_wall(
        &mut commands,
        Vec3::new(0.0, WINDOW_HEIGHT / 2.0 + 250.0, 0.0),
        Vec2::new(WINDOW_WIDTH, 500.0),
    );

    spawn_wall(
        &mut commands,
        Vec3::new(-WINDOW_WIDTH / 2.0 - 250.0, 0.0, 0.0),
        Vec2::new(500.0, WINDOW_HEIGHT),
    );

    spawn_wall(
        &mut commands,
        Vec3::new(WINDOW_WIDTH / 2.0 + 250.0, 0.0, 0.0),
        Vec2::new(500.0, WINDOW_HEIGHT),
    );

    // score
    commands.spawn((
        // Create a TextBundle that has a Text with a list of sections.
        TextBundle::from_sections([
            TextSection::new(
                "SCORE: ",
                TextStyle {
                    // This font is loaded and will be used instead of the default font.
                    font: asset_server.load("fonts/Inter-Bold.ttf"),
                    font_size: 42.0,
                    color: Color::WHITE,
                },
            ),
            TextSection::from_style({
                TextStyle {
                    font: asset_server.load("fonts/Inter-Bold.ttf"),
                    font_size: 42.0,
                    color: Color::GOLD,
                    // If no font is specified, the default font (a minimal subset of FiraMono) will be used.
                    ..default()
                }
            }),
        ]),
        ScoreText,
    ));
}

fn spawn_wall(commands: &mut Commands, position: Vec3, size: Vec2) {
    commands.spawn((
        SpriteBundle {
            sprite: Sprite {
                color: Color::rgb(0.2, 0.7, 0.9),
                custom_size: Some(size),
                ..Default::default()
            },
            transform: Transform::from_translation(position),
            ..Default::default()
        },
        RigidBody::Static,
        Collider::cuboid(size.x, size.y),
        CollisionLayers::new([Layer::Ground], [Layer::Fruits, Layer::Combining]),
    ));
}


fn spawn_timer(
    mut commands: Commands,
    score: Res<Score>,
    asset_server: Res<AssetServer>,
    fruit_prefabs: Res<FruitPrefabs>,
    time: Res<Time>,
    mut timer: ResMut<SpawnTimer>,
    query: Query<(Entity, &Position), With<Spawner>>,
) {
    if timer.0.tick(time.delta()).just_finished() {
        let (entity, parent_position) = query.single();

        let mut rng = rand::thread_rng();

        // generate weights based on score and generate a random FruitType based on those weights
        // improving the chance of higher fruit type based on score,
        // only include Cherry, Strawberry, Grape, Dekopon, and Orange
        

        let weights = HashMap::<FruitType, f32>::from_iter([
            (FruitType::Cherry, 0.5),
            (FruitType::Strawberry, 0.5),
            (FruitType::Grape, 0.3),
            (FruitType::Dekopon, 0.1),
            (FruitType::Orange, 0.1)
        ]);

        

        let random_fruit: FruitType =
            FruitType::from_repr(rng.gen_range(0..5)).unwrap_or(FruitType::Cherry);

        let bundle = get_fruit_bundle(&asset_server, &fruit_prefabs, parent_position, random_fruit);

        let fruit = commands.spawn(bundle).id();
        commands
            .entity(fruit)
            .insert((RigidBody::Kinematic, Controllable, CollisionLayers::new([Layer::Ground], [Layer::Ground])));
        commands.entity(entity).insert(Holding(fruit));
    }
}

fn create_particle_effect(effects: &mut ResMut<Assets<EffectAsset>>, parent_transform: &Transform) -> ParticleEffectBundle {
    let mut color_gradient1 = Gradient::new();
    color_gradient1.add_key(0.25, Vec4::new(0.5, 0.5, 0.5, 1.0));
    color_gradient1.add_key(0.50, Vec4::new(4.0, 4.0, 0.0, 1.0));
    color_gradient1.add_key(0.75, Vec4::new(4.0, 0.0, 0.0, 1.0));
    color_gradient1.add_key(1.0, Vec4::new(4.0, 0.0, 0.0, 0.0));

    let mut size_gradient1 = Gradient::new();
    size_gradient1.add_key(0.3, Vec2::new(1.0, 1.0));
    size_gradient1.add_key(1.0, Vec2::splat(0.0));

    let writer = ExprWriter::new();

    let init_pos = SetPositionCircleModifier {
        center: writer.lit(Vec3::ZERO).expr(),
        axis: writer.lit(Vec3::Z).expr(),
        radius: writer.lit(30.).expr(),
        dimension: ShapeDimension::Surface,
    };

    let age = writer.lit(0.).expr();
    let init_age = SetAttributeModifier::new(Attribute::AGE, age);

    // Give a bit of variation by randomizing the lifetime per particle
    let lifetime = writer.lit(0.5).uniform(writer.lit(15.0)).expr();
    let init_lifetime = SetAttributeModifier::new(Attribute::LIFETIME, lifetime);

    // Add drag to make particles slow down a bit after the initial acceleration
    let drag = writer.lit(2.).expr();
    let update_drag = LinearDragModifier::new(drag);


    let mut module = writer.finish();

    let tangent_accel = TangentAccelModifier::constant(&mut module, Vec3::ZERO, Vec3::Z, 50.);

    let effect1 = effects.add(
        EffectAsset::new(32768, bevy_hanabi::Spawner::rate(5000.0.into()), module)
            .with_name("portal")
            .init(init_pos)
            .init(init_age)
            .init(init_lifetime)
            .update(update_drag)
            .update(tangent_accel)
            .render(ColorOverLifetimeModifier {
                gradient: color_gradient1,
            })
            .render(SizeOverLifetimeModifier {
                gradient: size_gradient1,
                screen_space_size: true,
            })
            .render(OrientModifier {
                mode: OrientMode::AlongVelocity,
            }),
    );

    return ParticleEffectBundle {
        effect: ParticleEffect::new(effect1).with_z_layer_2d(Some(0.1)),
        transform: parent_transform.clone(),
        ..Default::default()
    }
}

#[derive(PhysicsLayer)]
enum Layer {
    Ground,
    Fruits,
    Combining,
}

static DEFAULT_FRUIT_LAYER: Lazy<CollisionLayers> = Lazy::new(|| CollisionLayers::new([Layer::Fruits], [Layer::Fruits, Layer::Ground]));

fn get_fruit_bundle(
    asset_server: &Res<AssetServer>,
    fruit_prefabs: &Res<FruitPrefabs>,
    parent_position: &Position,
    fruit: FruitType,
) -> FruitBundle {
    // Get the fruit prefab
    let fruit_prefab = fruit_prefabs
        .map
        .get(&fruit)
        .expect(&format!("NO FRUIT! {}", fruit.to_string()));

    // Spawn the fruit entity
    return FruitBundle {
        sprite: SpriteBundle {
            transform: Transform::from_translation(parent_position.0.extend(0.0)),
            texture: get_fruit_texture(asset_server, &fruit),
            sprite: Sprite {
                custom_size: Some(Vec2 {
                    x: fruit_prefab.size.clone(),
                    y: fruit_prefab.size.clone(),
                }),
                ..Default::default()
            },
            ..Default::default()
        },
        fruit: Fruit(fruit),
        restitution: Restitution::new(0.2),
        collider: Collider::ball(fruit_prefab.size / 2.0),
        collision_layers: *DEFAULT_FRUIT_LAYER,
    };
}

fn get_fruit_texture(asset_server: &Res<AssetServer>, fruit_type: &FruitType) -> Handle<Image> {
    return asset_server.load::<Image, String>(format!("{}.png", fruit_type.to_string()));
}

fn spawner(
    mut commands: Commands,
    spawners: Query<(Entity, (With<Spawner>, Without<Holding>, Without<Spawning>))>,
) {
    let Ok((entity, _)) = spawners.get_single() else {
        return;
    };

    // not holding a fruit, insert a resource to spawn a fruit

    commands.entity(entity).insert(Spawning);
    commands.insert_resource(SpawnTimer(Timer::from_seconds(0.5, TimerMode::Once)));
}

fn grow(
    mut commands: Commands,
    time: Res<Time>,
    asset_server: Res<AssetServer>,
    fruit_prefabs: Res<FruitPrefabs>,
    query: Query<(Entity, &Fruit, &Position, &Growing)>,
) {
    for (entity, fruit, parent_position, Growing(combine_start, current_fruit)) in query.iter() {
        let next_fruit = FruitType::from_repr(cmp::min(
            *current_fruit as u8 + 1,
            FruitType::COUNT.try_into().unwrap(),
        )).unwrap();

        let mut bundle = get_fruit_bundle(
            &asset_server,
            &fruit_prefabs,
            parent_position,
            next_fruit,
        );

        let old_size = fruit_prefabs.map.get(current_fruit).unwrap().size;

        // maybe return to the old way, and not precalcuate the fruit
        let new_size = fruit_prefabs.map.get(&next_fruit).unwrap().size;


        // grow over 500ms
        println!("Time: {:?}", *combine_start);
        let progress: f32 = (time.elapsed() - *combine_start).as_millis() as f32/ 500.0;
        let lerp_size = lerp(old_size, new_size, progress);

        assert!(lerp_size <= new_size);

        println!("From: {:?}, To: {:?}, At: {:?}, Progress: {}", old_size, new_size, lerp_size, progress);

        // finished growing, remove growing
        if lerp_size == new_size {
            commands.entity(entity).remove::<Growing>();

            println!("{:?}", (fruit_prefabs.map));
        }

        bundle.collider = Collider::ball(lerp_size / 2.0);

        // WHY NO ENTITY? TWO ENTITIES COMBINING WITH EACH OTHER?????
        let Some(mut entity_commands) = commands.get_entity(entity) else {
            continue;
        };

        entity_commands
            .insert(bundle)
            .insert(RigidBody::Dynamic);
    }
}

fn combining(
    time: Res<Time>,
    mut commands: Commands,
    fruit_prefabs: Res<FruitPrefabs>,
    mut score: ResMut<Score>,
    mut query: Query<(Entity, &Fruit, &Combining)>,
    mut transform_query: Query<&mut Transform, With<LinearVelocity>>,
) {
    for (entity, fruit, combining) in &mut query.iter_mut() {
        assert!(entity != combining.0);
        let Ok([mut source_transform, target_transform]) =
            transform_query.get_many_mut([entity, combining.0])
        else {
            continue;
        };

        let position1 = source_transform.translation.truncate();
        let position2 = target_transform.translation.truncate();

        let progress: f64 = (time.elapsed() - combining.2).as_millis() as f64
            / 250.0;

        source_transform.translation = combining.1.0.lerp(position2, progress.min(1.0) as f32).extend(0.0);

        commands.entity(entity).insert(LinearVelocity::ZERO);

        let distance = Vec2::distance(position1, position2);

        if distance < 1.0 {
            commands.entity(entity).despawn();
            commands.entity(combining.0).insert(Growing(time.elapsed(), fruit.0.clone()));
            score.0 += fruit_prefabs.map.get(&fruit.0).unwrap().points as i64;
        }

        // original already was combined, need to make combining unique somehow...
        if commands.get_entity(combining.0).is_none() {
            // this is no good, need to restore
            commands.entity(entity).remove::<Combining>();
        }
    }
}

fn collision(
    mut commands: Commands,
    time: Res<Time>,
    collisions: Res<Collisions>,
    mut effects: ResMut<Assets<EffectAsset>>,

    // colliding_entities: Query<(Entity, &CollidingEntities), Without<Combining>>,
    query: Query<(Entity, &Fruit, &Position), Without<Combining>>,
) {
    for contacts in collisions.iter() {
        if !contacts.during_current_substep {
            continue;
        }

        if let Ok([(entity1, fruit1, pos1), (entity2, fruit2, pos2)]) =
            query.get_many([contacts.entity1, contacts.entity2])
        {
            if fruit1.0 == fruit2.0 {
                let (survivor, killed, killed_pos) = match pos1.y < pos2.y {
                    true => (entity1, entity2, pos2),
                    false => (entity2, entity1, pos1),
                };

                commands
                    .entity(killed)
                    .insert((
                        create_particle_effect(&mut effects, &Transform::from_translation(pos1.0.extend(0.0))),
                        Combining(survivor, Position(killed_pos.0.clone()), time.elapsed()),
                        RigidBody::Kinematic,
                        ExternalForce::ZERO,
                        ExternalTorque::ZERO,
                    ))
                    .remove::<Collider>();
            }
        }
    }
}

fn input(
    mut commands: Commands,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    q_windows: Query<&Window, With<PrimaryWindow>>,
    button_input: Res<Input<MouseButton>>,
    mut player: Query<(Entity, &mut Position), With<Controllable>>,
    holding: Query<(Entity, &mut Holding)>,
) {
    if button_input.pressed(MouseButton::Left) {
        let (camera, camera_transform) = camera_query.single();

        let Some(cursor_position) = q_windows.single().cursor_position() else {
            return;
        };

        let Some(point) = camera.viewport_to_world_2d(camera_transform, cursor_position) else {
            return;
        };

        for (_, mut position) in player.iter_mut() {
            position.x = point.x;
            position.y = (WINDOW_HEIGHT / 2.0) - 50.0;
        }
    }

    if button_input.just_released(MouseButton::Left) {
        let Ok((entity, holding)) = holding.get_single() else {
            return;
        };
        // drop
        // do the drop?

        let Some(mut holding_entity) = commands.get_entity(holding.0) else {
            return;
        };

        holding_entity
            .insert(RigidBody::Dynamic)
            .insert(Restitution::new(0.0))
            .insert(*DEFAULT_FRUIT_LAYER)
            .remove::<Controllable>();

        commands
            .entity(entity)
            .remove::<Holding>()
            .remove::<Spawning>();

        // Query holding, get the entity, do these things, then remove holding from player

        // too much logic in here, i don't like that.
    }
}
