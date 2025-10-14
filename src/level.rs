//! Functions related to spawning levels.

use crate::{
    app::{
        LdtkEntity, LdtkEntityMapBackend, LdtkIntCellMapBackend, PhantomLdtkEntity,
        PhantomLdtkEntityTrait, PhantomLdtkIntCell, PhantomLdtkIntCellTrait,
    },
    components::*,
    ldtk::{
        loaded_level::LoadedLevel, EntityDefinition, EnumTagValue, LayerDefinition, LayerInstance,
        LevelBackgroundPosition, TileCustomMetadata, TileInstance, TilesetDefinition, Type,
    },
    resources::{IntGridRendering, LdtkSettings, LevelBackground},
    tile_makers::{
        tile_pos_to_invisible_tile, tile_pos_to_tile_grid_bundle_maker,
        tile_pos_to_tile_if_int_grid_nonzero_maker, tile_pos_to_transparent_tile_maker,
    },
    utils::*,
};

use bevy::{
    ecs::{
        entity::{EntityMapper, MapEntities},
        reflect::ReflectMapEntities,
    },
    prelude::*,
};
// use bevy_ecs_tilemap::{
//     map::{
//         TilemapGridSize, TilemapId, TilemapSize, TilemapSpacing, TilemapTexture, TilemapTileSize,
//     },
//     tiles::{TilePos, TileStorage},
// };
use std::collections::{HashMap, HashSet};

// #[cfg(feature = "render")]
// use bevy_ecs_tilemap::TilemapBundle;

// #[cfg(not(feature = "render"))]
// use bevy_ecs_tilemap::StandardTilemapBundle as TilemapBundle;

use thiserror::Error;

/// The default tilemap bundle. All of the components within are required.
#[derive(Bundle, Debug, Default, Clone)]
pub struct TilemapBundle {
    pub size: TilemapSize,
    pub storage: TileStorage,
}

/// A component which stores a reference to the tilemap entity.
#[derive(Component, Reflect, Clone, Copy, Debug, Hash, Deref, DerefMut, PartialEq, Eq)]
#[reflect(Component, MapEntities)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TilemapId(pub Entity);

impl MapEntities for TilemapId {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.0 = entity_mapper.get_mapped(self.0);
    }
}

impl Default for TilemapId {
    fn default() -> Self {
        Self(Entity::from_raw_u32(0).expect("bsREASON"))
    }
}

/// Hides or shows a tile based on the boolean. Default: True
#[derive(Component, Reflect, Clone, Copy, Debug, Hash, PartialEq, Eq)]
#[reflect(Component)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TileVisible(pub bool);

impl Default for TileVisible {
    fn default() -> Self {
        Self(true)
    }
}

/// This an optional tile bundle with default components.
#[derive(Bundle, Default, Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TileBundle {
    pub position: TilePos,
    pub tilemap_id: TilemapId,
    pub visible: TileVisible,
}

#[derive(Component, Reflect, Default, Clone, Copy, Debug, Hash, PartialEq)]
#[reflect(Component)]
pub struct TilemapSize {
    pub(crate) x: u32,
    pub(crate) y: u32,
}

impl TilemapSize {
    pub const fn new(x: u32, y: u32) -> Self {
        Self { x, y }
    }

    pub const fn count(&self) -> usize {
        (self.x * self.y) as usize
    }
}

#[derive(Component, Reflect, Default, Debug, Clone)]
#[reflect(Component, MapEntities)]
pub struct TileStorage {
    tiles: Vec<Option<Entity>>,
    pub size: TilemapSize,
}

impl MapEntities for TileStorage {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        for entity in self.tiles.iter_mut().flatten() {
            *entity = entity_mapper.get_mapped(*entity);
        }
    }
}

impl TileStorage {
    /// Creates a new tile storage that is empty.
    pub fn empty(size: TilemapSize) -> Self {
        Self {
            tiles: vec![None; size.count()],
            size,
        }
    }

    /// Gets a tile entity for the given tile position, if an entity is associated with that tile
    /// position.
    ///
    /// Panics if the given `tile_pos` doesn't lie within the extents of the underlying tile map.
    pub fn get(&self, tile_pos: &TilePos) -> Option<Entity> {
        self.tiles[tile_pos.to_index(&self.size)]
    }

    /// Gets a tile entity for the given tile position, if:
    /// 1) the tile position lies within the underlying tile map's extents *and*
    /// 2) there is an entity associated with that tile position;
    ///
    /// otherwise it returns `None`.
    pub fn checked_get(&self, tile_pos: &TilePos) -> Option<Entity> {
        if tile_pos.within_map_bounds(&self.size) {
            self.tiles[tile_pos.to_index(&self.size)]
        } else {
            None
        }
    }

    /// Sets a tile entity for the given tile position.
    ///
    /// If there is an entity already at that position, it will be replaced.
    ///
    /// Panics if the given `tile_pos` doesn't lie within the extents of the underlying tile map.
    pub fn set(&mut self, tile_pos: &TilePos, tile_entity: Entity) {
        self.tiles[tile_pos.to_index(&self.size)].replace(tile_entity);
    }

    /// Sets a tile entity for the given tile position, if the tile position lies within the
    /// underlying tile map's extents.
    ///
    /// If there is an entity already at that position, it will be replaced.
    pub fn checked_set(&mut self, tile_pos: &TilePos, tile_entity: Entity) {
        if tile_pos.within_map_bounds(&self.size) {
            self.tiles[tile_pos.to_index(&self.size)].replace(tile_entity);
        }
    }

    /// Returns an iterator with all of the positions in the grid.
    pub fn iter(&self) -> impl Iterator<Item = &Option<Entity>> {
        self.tiles.iter()
    }

    /// Returns mutable iterator with all of the positions in the grid.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Option<Entity>> {
        self.tiles.iter_mut()
    }

    /// Removes any stored `Entity` at the given tile position, leaving `None` in its place and
    /// returning the `Entity`.
    ///
    /// Panics if the given `tile_pos` doesn't lie within the extents of the underlying tile map.
    pub fn remove(&mut self, tile_pos: &TilePos) -> Option<Entity> {
        self.tiles[tile_pos.to_index(&self.size)].take()
    }

    /// Remove any stored `Entity` at the given tile position, leaving `None` in its place and
    /// returning the `Entity`.
    ///
    /// Checks that the given `tile_pos` lies within the extents of the underlying map.
    pub fn checked_remove(&mut self, tile_pos: &TilePos) -> Option<Entity> {
        self.tiles.get_mut(tile_pos.to_index(&self.size))?.take()
    }

    /// Removes all stored `Entity`s, leaving `None` in their place and
    /// returning them in an iterator.
    ///
    /// Example:
    /// ```
    /// # use bevy::prelude::Commands;
    /// # use bevy_ecs_tilemap::prelude::{TilemapSize, TileStorage};
    /// # fn example(mut commands: Commands) {
    /// # let mut storage = TileStorage::empty(TilemapSize { x: 16, y: 16 });
    /// for entity in storage.drain() {
    ///   commands.entity(entity).despawn();
    /// }
    /// # }
    /// ```
    pub fn drain(&mut self) -> impl Iterator<Item = Entity> + use<'_> {
        self.tiles.iter_mut().filter_map(|opt| opt.take())
    }
}

/// A tile position in the tilemap grid.
#[derive(Component, Reflect, Default, Clone, Copy, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
#[reflect(Component)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TilePos {
    pub x: u32,
    pub y: u32,
}

impl TilePos {
    pub const fn new(x: u32, y: u32) -> Self {
        Self { x, y }
    }

    /// Converts a tile position (2D) into an index in a flattened vector (1D), assuming the
    /// tile position lies in a tilemap of the specified size.
    pub fn to_index(&self, tilemap_size: &TilemapSize) -> usize {
        ((self.y * tilemap_size.x) + self.x) as usize
    }

    /// Checks to see if `self` lies within a tilemap of the specified size.
    pub fn within_map_bounds(&self, map_size: &TilemapSize) -> bool {
        self.x < map_size.x && self.y < map_size.y
    }
}

#[derive(Error, Debug)]
enum BackgroundImageError {
    #[error("background image handle not loaded into the image assets store")]
    ImageNotLoaded,
}

fn background_image_sprite_sheet(
    images: &Assets<Image>,
    texture_atlases: &mut Assets<TextureAtlasLayout>,
    background_image_handle: &Handle<Image>,
    background_position: &LevelBackgroundPosition,
    level_height: i32,
    transform_z: f32,
) -> Result<(Sprite, Transform), BackgroundImageError> {
    if let Some(background_image) = images.get(background_image_handle) {
        // We need to use a texture atlas to apply the correct crop to the image
        let tile_size = UVec2::new(
            background_image.texture_descriptor.size.width,
            background_image.texture_descriptor.size.height,
        );
        let mut texture_atlas_layout = TextureAtlasLayout::new_empty(tile_size);

        let min = Vec2::new(
            background_position.crop_rect[0],
            background_position.crop_rect[1],
        );

        let size = Vec2::new(
            background_position.crop_rect[2],
            background_position.crop_rect[3],
        );

        let max = min + size;

        let crop_rect = Rect { min, max };

        let index = texture_atlas_layout.add_texture(crop_rect.as_urect());

        let scale = background_position.scale;

        let scaled_size = size * scale;

        let top_left_translation =
            ldtk_pixel_coords_to_translation(background_position.top_left_px, level_height);

        let center_translation =
            top_left_translation + (Vec2::new(scaled_size.x, -scaled_size.y) / 2.);

        Ok((
            Sprite::from_atlas_image(
                background_image_handle.clone(),
                TextureAtlas {
                    index,
                    layout: texture_atlases.add(texture_atlas_layout),
                },
            ),
            Transform::from_translation(center_translation.extend(transform_z))
                .with_scale(scale.extend(1.)),
        ))
    } else {
        Err(BackgroundImageError::ImageNotLoaded)
    }
}

pub(crate) fn tile_to_grid_coords(
    tile_instance: &TileInstance,
    layer_height_in_tiles: i32,
    layer_grid_size: i32,
) -> GridCoords {
    ldtk_pixel_coords_to_grid_coords(
        IVec2::new(tile_instance.px[0], tile_instance.px[1]),
        layer_height_in_tiles,
        IVec2::splat(layer_grid_size),
    )
}

fn insert_metadata_to_tile(
    commands: &mut Commands,
    tile_instance: &TileInstance,
    tile_entity: Entity,
    metadata_map: &HashMap<i32, TileMetadata>,
    enum_tags_map: &HashMap<i32, TileEnumTags>,
) -> bool {
    let mut entity_commands = commands.entity(tile_entity);

    let mut metadata_inserted = false;

    if let Some(tile_metadata) = metadata_map.get(&tile_instance.t) {
        entity_commands.insert(tile_metadata.clone());
        metadata_inserted = true;
    }

    if let Some(enum_tags) = enum_tags_map.get(&tile_instance.t) {
        entity_commands.insert(enum_tags.clone());
        metadata_inserted = true;
    }

    metadata_inserted
}

fn spatial_bundle_for_tiles(grid_coords: GridCoords, grid_size: i32) -> Transform {
    let translation =
        grid_coords_to_translation_relative_to_tile_layer(grid_coords, IVec2::splat(grid_size))
            .extend(0.);

    Transform::from_translation(translation)
}

fn insert_spatial_bundle_for_layer_tiles(
    commands: &mut Commands,
    storage: &TileStorage,
    size: &TilemapSize,
    grid_size: i32,
    tilemap_id: TilemapId,
) {
    for x in 0..size.x {
        for y in 0..size.y {
            let tile_pos = TilePos { x, y };
            let tile_entity = storage.get(&tile_pos);

            if let Some(tile_entity) = tile_entity {
                let spatial_bundle = spatial_bundle_for_tiles(tile_pos.into(), grid_size);

                commands.entity(tile_entity).insert(spatial_bundle);
                commands.entity(tilemap_id.0).add_child(tile_entity);
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn insert_tile_metadata_for_layer(
    commands: &mut Commands,
    tile_storage: &TileStorage,
    grid_tiles: &[TileInstance],
    layer_instance: &LayerInstance,
    metadata_map: &HashMap<i32, TileMetadata>,
    enum_tags_map: &HashMap<i32, TileEnumTags>,
) {
    for tile in grid_tiles {
        let grid_coords = tile_to_grid_coords(tile, layer_instance.c_hei, layer_instance.grid_size);

        let tile_entity = tile_storage.get(&grid_coords.into()).unwrap();

        insert_metadata_to_tile(commands, tile, tile_entity, metadata_map, enum_tags_map);
    }
}

fn layer_grid_tiles(grid_tiles: Vec<TileInstance>) -> Vec<Vec<TileInstance>> {
    let mut layer = Vec::new();
    let mut overflow = Vec::new();
    for tile in grid_tiles {
        if layer.iter().any(|t: &TileInstance| t.px == tile.px) {
            overflow.push(tile);
        } else {
            layer.push(tile);
        }
    }

    let mut layered_grid_tiles = vec![layer];
    if !overflow.is_empty() {
        layered_grid_tiles.extend(layer_grid_tiles(overflow));
    }

    layered_grid_tiles
}

fn tile_in_layer_bounds(tile: &TileInstance, layer_instance: &LayerInstance) -> bool {
    tile.px.x >= 0
        && tile.px.y >= 0
        && tile.px.x < (layer_instance.c_wid * layer_instance.grid_size)
        && tile.px.y < (layer_instance.c_hei * layer_instance.grid_size)
}

#[allow(clippy::too_many_arguments)]
pub fn spawn_level(
    level: LoadedLevel,
    background_image: &Option<Handle<Image>>,
    commands: &mut Commands,
    asset_server: &AssetServer,
    images: &Assets<Image>,
    texture_atlases: &mut Assets<TextureAtlasLayout>,
    ldtk_entity_map: &LdtkEntityMapBackend,
    ldtk_int_cell_map: &LdtkIntCellMapBackend,
    entity_definition_map: &HashMap<i32, &EntityDefinition>,
    layer_definition_map: &HashMap<i32, &LayerDefinition>,
    tileset_map: &HashMap<i32, Handle<Image>>,
    tileset_definition_map: &HashMap<i32, &TilesetDefinition>,
    int_grid_image_handle: &Option<Handle<Image>>,
    worldly_set: HashSet<Worldly>,
    ldtk_entity: Entity,
    ldtk_settings: &LdtkSettings,
) {
    let layer_instances = level.layer_instances();

    let mut layer_z = 0;

    // if ldtk_settings.level_background == LevelBackground::Rendered {
    //     let translation = Vec3::new(*level.px_wid() as f32, *level.px_hei() as f32, 0.) / 2.;

    //     let background_entity = commands
    //         .spawn((
    //             Sprite {
    //                 color: *level.bg_color(),
    //                 custom_size: Some(Vec2::new(*level.px_wid() as f32, *level.px_hei() as f32)),
    //                 ..default()
    //             },
    //             Transform::from_translation(translation),
    //         ))
    //         .id();

    //     commands.entity(ldtk_entity).add_child(background_entity);

    //     layer_z += 1;

    //     // Spawn background image
    //     if let (Some(background_image_handle), Some(background_position)) =
    //         (background_image, level.bg_pos())
    //     {
    //         match background_image_sprite_sheet(
    //             images,
    //             texture_atlases,
    //             background_image_handle,
    //             background_position,
    //             *level.px_hei(),
    //             layer_z as f32,
    //         ) {
    //             Ok(sprite_sheet) => {
    //                 commands.entity(ldtk_entity).with_children(|parent| {
    //                     parent.spawn(sprite_sheet);
    //                 });

    //                 layer_z += 1;
    //             }
    //             Err(e) => warn!("{}", e),
    //         }
    //     }
    // }

    for layer_instance in layer_instances
        .iter()
        .filter(|layer| {
            !ldtk_settings
                .exclusions
                .layer_identifiers
                .contains(&layer.identifier)
        })
        .rev()
    {
        let layer_offset = Vec2::new(
            layer_instance.px_total_offset_x as f32,
            -layer_instance.px_total_offset_y as f32,
        );

        match layer_instance.layer_instance_type {
            Type::Entities => {
                let layer_entity = commands
                    .spawn(Transform::from_translation(
                        layer_offset.extend(layer_z as f32),
                    ))
                    .insert(Visibility::default())
                    .insert(LayerMetadata::from(layer_instance))
                    .insert(Name::new(layer_instance.identifier.to_owned()))
                    .with_children(|commands| {
                        for entity_instance in &layer_instance.entity_instances {
                            let transform = calculate_transform_from_entity_instance(
                                entity_instance,
                                entity_definition_map,
                                *level.px_hei(),
                            );
                            // Note: entities do not seem to be affected visually by layer offsets in
                            // the editor, so no layer offset is added to the transform here.

                            let (tileset, tileset_definition) = match &entity_instance.tile {
                                Some(t) => (
                                    tileset_map.get(&t.tileset_uid),
                                    tileset_definition_map.get(&t.tileset_uid).copied(),
                                ),
                                None => (None, None),
                            };

                            let predicted_worldly = Worldly::bundle_entity(
                                entity_instance,
                                layer_instance,
                                tileset,
                                tileset_definition,
                                asset_server,
                                texture_atlases,
                            );

                            if !worldly_set.contains(&predicted_worldly) {
                                let default_ldtk_entity: Box<dyn PhantomLdtkEntityTrait> =
                                    Box::new(PhantomLdtkEntity::<EntityInstanceBundle>::new());
                                let mut entity_commands = commands.spawn_empty();

                                // insert Name before evaluating LdtkEntitys so that user-provided
                                // names aren't overwritten
                                entity_commands.insert((
                                    EntityIid::new(entity_instance.iid.to_owned()),
                                    Name::new(entity_instance.identifier.to_owned()),
                                ));

                                ldtk_map_get_or_default(
                                    layer_instance.identifier.clone(),
                                    entity_instance.identifier.clone(),
                                    &default_ldtk_entity,
                                    ldtk_entity_map,
                                )
                                .evaluate(
                                    &mut entity_commands,
                                    entity_instance,
                                    layer_instance,
                                    tileset,
                                    tileset_definition,
                                    asset_server,
                                    texture_atlases,
                                );

                                entity_commands.insert(transform);
                            }
                        }
                    })
                    .id();

                commands.entity(ldtk_entity).add_child(layer_entity);
                layer_z += 1;
            }
            _ => {
                // The remaining layers have a lot of shared code.
                // This is because:
                // 1. There is virtually no difference between AutoTile and Tile layers
                // 2. IntGrid layers can sometimes have AutoTile functionality

                let size = TilemapSize {
                    x: layer_instance.c_wid as u32,
                    y: layer_instance.c_hei as u32,
                };

                let tileset_definition = layer_instance
                    .tileset_def_uid
                    .map(|u| tileset_definition_map.get(&u).unwrap());

                let tile_size = tileset_definition
                    .map(|TilesetDefinition { tile_grid_size, .. }| *tile_grid_size)
                    .unwrap_or(layer_instance.grid_size) as f32;

                // let tilemap_tile_size = TilemapTileSize {
                //     x: tile_size,
                //     y: tile_size,
                // };

                let grid_size = layer_instance.grid_size as f32;

                // let tilemap_grid_size = TilemapGridSize {
                //     x: grid_size,
                //     y: grid_size,
                // };

                // let spacing = match tileset_definition {
                //     Some(tileset_definition) if tileset_definition.spacing != 0 => {
                //         // TODO: Check that this is still an issue with upcoming
                //         // bevy_ecs_tilemap releases
                //         #[cfg(not(feature = "atlas"))]
                //         {
                //             warn!(
                //                         "Tile spacing on Tile and AutoTile layers requires the \"atlas\" feature"
                //                     );

                //             TilemapSpacing::default()
                //         }

                //         #[cfg(feature = "atlas")]
                //         {
                //             TilemapSpacing {
                //                 x: tileset_definition.spacing as f32,
                //                 y: tileset_definition.spacing as f32,
                //             }
                //         }
                //     }
                //     _ => TilemapSpacing::default(),
                // };

                // let texture = match (tileset_definition, int_grid_image_handle) {
                //     (Some(tileset_definition), _) => TilemapTexture::Single(
                //         tileset_map.get(&tileset_definition.uid).unwrap().clone(),
                //     ),
                //     (None, Some(handle)) => TilemapTexture::Single(handle.clone()),
                //     _ => {
                //         warn!("unable to render tilemap layer, it has no tileset and no intgrid layers were expected");
                //         continue;
                //     }
                // };

                let metadata_map: HashMap<i32, TileMetadata> = tileset_definition
                    .map(|tileset_definition| {
                        tileset_definition
                            .custom_data
                            .iter()
                            .map(|TileCustomMetadata { data, tile_id }| {
                                (*tile_id, TileMetadata { data: data.clone() })
                            })
                            .collect()
                    })
                    .unwrap_or_default();

                let mut enum_tags_map: HashMap<i32, TileEnumTags> = HashMap::new();

                if let Some(tileset_definition) = tileset_definition {
                    for EnumTagValue {
                        enum_value_id,
                        tile_ids,
                    } in tileset_definition.enum_tags.iter()
                    {
                        for tile_id in tile_ids {
                            enum_tags_map
                                .entry(*tile_id)
                                .or_insert_with(|| TileEnumTags {
                                    tags: Vec::new(),
                                    source_enum_uid: tileset_definition.tags_source_enum_uid,
                                })
                                .tags
                                .push(enum_value_id.clone());
                        }
                    }
                }

                let mut grid_tiles = layer_instance.grid_tiles.clone();
                grid_tiles.extend(layer_instance.auto_layer_tiles.clone());

                for (i, grid_tiles) in layer_grid_tiles(grid_tiles)
                    .into_iter()
                    // filter out tiles that are out of bounds
                    .map(|grid_tiles| {
                        grid_tiles
                            .into_iter()
                            .filter(|tile| tile_in_layer_bounds(tile, layer_instance))
                            .collect::<Vec<_>>()
                    })
                    .enumerate()
                {
                    let layer_entity = commands.spawn_empty().id();

                    let tilemap_bundle = if layer_instance.layer_instance_type == Type::IntGrid {
                        // The current spawning of IntGrid layers doesn't allow using
                        // LayerBuilder::new_batch().
                        // So, the actual LayerBuilder usage diverges greatly here
                        let mut storage = TileStorage::empty(size);

                        match tileset_definition {
                            Some(_) => {
                                // set_all_tiles_with_func(
                                //     commands,
                                //     &mut storage,
                                //     size,
                                //     TilemapId(layer_entity),
                                //     tile_pos_to_tile_grid_bundle_maker(
                                //         tile_pos_to_transparent_tile_maker(
                                //             tile_pos_to_int_grid_with_grid_tiles_tile_maker(
                                //                 &grid_tiles,
                                //                 &layer_instance.int_grid_csv,
                                //                 layer_instance.c_wid,
                                //                 layer_instance.c_hei,
                                //                 layer_instance.grid_size,
                                //                 i,
                                //             ),
                                //             layer_instance.opacity,
                                //         ),
                                //     ),
                                // );
                            }
                            None => {
                                let int_grid_value_defs = &layer_definition_map
                                    .get(&layer_instance.layer_def_uid)
                                    .expect("Encountered layer without definition")
                                    .int_grid_values;

                                match ldtk_settings.int_grid_rendering {
                                    IntGridRendering::Colorful => {
                                        // set_all_tiles_with_func(
                                        //     commands,
                                        //     &mut storage,
                                        //     size,
                                        //     TilemapId(layer_entity),
                                        //     tile_pos_to_tile_grid_bundle_maker(
                                        //         tile_pos_to_transparent_tile_maker(
                                        //             tile_pos_to_int_grid_colored_tile_maker(
                                        //                 &layer_instance.int_grid_csv,
                                        //                 int_grid_value_defs,
                                        //                 layer_instance.c_wid,
                                        //                 layer_instance.c_hei,
                                        //             ),
                                        //             layer_instance.opacity,
                                        //         ),
                                        //     ),
                                        // );
                                    }
                                    IntGridRendering::Invisible => {
                                        set_all_tiles_with_func(
                                            commands,
                                            &mut storage,
                                            size,
                                            TilemapId(layer_entity),
                                            tile_pos_to_tile_grid_bundle_maker(
                                                tile_pos_to_transparent_tile_maker(
                                                    tile_pos_to_tile_if_int_grid_nonzero_maker(
                                                        tile_pos_to_invisible_tile,
                                                        &layer_instance.int_grid_csv,
                                                        layer_instance.c_wid,
                                                        layer_instance.c_hei,
                                                    ),
                                                    layer_instance.opacity,
                                                ),
                                            ),
                                        );
                                    }
                                }
                            }
                        }

                        if i == 0 {
                            for (i, value) in layer_instance
                                .int_grid_csv
                                .iter()
                                .enumerate()
                                .filter(|(_, v)| **v != 0)
                            {
                                let grid_coords = int_grid_index_to_grid_coords(
                                        i,
                                        layer_instance.c_wid as u32,
                                        layer_instance.c_hei as u32,
                                    ).expect("int_grid_csv indices should be within the bounds of 0..(layer_width * layer_height)");

                                if let Some(tile_entity) = storage.get(&grid_coords.into()) {
                                    let mut entity_commands = commands.entity(tile_entity);

                                    let default_ldtk_int_cell: Box<dyn PhantomLdtkIntCellTrait> =
                                        Box::new(PhantomLdtkIntCell::<IntGridCellBundle>::new());

                                    ldtk_map_get_or_default(
                                        layer_instance.identifier.clone(),
                                        *value,
                                        &default_ldtk_int_cell,
                                        ldtk_int_cell_map,
                                    )
                                    .evaluate(
                                        &mut entity_commands,
                                        IntGridCell { value: *value },
                                        layer_instance,
                                    );
                                }
                            }
                        }

                        if !(metadata_map.is_empty() && enum_tags_map.is_empty()) {
                            insert_tile_metadata_for_layer(
                                commands,
                                &storage,
                                &grid_tiles,
                                layer_instance,
                                &metadata_map,
                                &enum_tags_map,
                            );
                        }

                        TilemapBundle {
                            size,
                            storage,
                            ..default()
                        }
                    } else {
                        let storage = TileStorage::empty(size);

                        TilemapBundle { size, storage }
                    };

                    insert_spatial_bundle_for_layer_tiles(
                        commands,
                        &tilemap_bundle.storage,
                        &tilemap_bundle.size,
                        layer_instance.grid_size,
                        TilemapId(layer_entity),
                    );

                    let LayerDefinition {
                        tile_pivot_x,
                        tile_pivot_y,
                        ..
                    } = &layer_definition_map
                        .get(&layer_instance.layer_def_uid)
                        .expect("Encountered layer without definition");

                    // The math for determining the x/y of a tilemap layer depends heavily on
                    // both the layer's grid size and the tileset's tile size.
                    // In particular, we care about their difference for properly reversing y
                    // direction and for tile pivot calculations.
                    let grid_tile_size_difference = grid_size - tile_size;

                    // It is useful to determine what we should treat as the desired "origin" of
                    // the tilemap in bevy space.
                    // This will be the bottom left pixel of the tilemap.
                    // The y value is affected when there is a difference between the grid size and
                    // tile size - it sinks below 0 when the grid size is greater.
                    let bottom_left_pixel = Vec2::new(0., grid_tile_size_difference);

                    // Tiles in bevy_ecs_tilemap are anchored to the center of the tile.
                    // We need to cancel out this anchoring so that layers of different sizes will
                    // stack on top of eachother as they do in LDtk.
                    let centering_adjustment = Vec2::splat(tile_size / 2.);

                    // Layers in LDtk can have a pivot value that acts like an anchor.
                    // The amount that a tile is translated by this pivot is simply the difference
                    // between grid_size and tile_size again.
                    let pivot_adjustment = Vec2::new(
                        grid_tile_size_difference * tile_pivot_x,
                        -grid_tile_size_difference * tile_pivot_y,
                    );

                    commands
                        .entity(layer_entity)
                        .insert(tilemap_bundle)
                        .insert(Transform::from_translation(
                            (bottom_left_pixel
                                + centering_adjustment
                                + pivot_adjustment
                                + layer_offset)
                                .extend(layer_z as f32),
                        ))
                        .insert(Visibility::default())
                        .insert(LayerMetadata::from(layer_instance))
                        .insert(Name::new(layer_instance.identifier.to_owned()));

                    commands.entity(ldtk_entity).add_child(layer_entity);

                    layer_z += 1;
                }
            }
        }
    }
}
