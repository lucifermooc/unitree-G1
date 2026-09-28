PRAGMA foreign_keys= OFF;
BEGIN TRANSACTION;
CREATE TABLE map
(
    id               integer primary key autoincrement,
    name             text,
    file_path        text,
    create_timestamp int,
    edit_timestamp   int,
    creator_id       int
);
CREATE TABLE waypoint
(
    id               integer primary key autoincrement,
    map_id           integer,
    point_list       text,
    frame_id         text,
    create_timestamp int,
    edit_timestamp   int,
    creator_id       int
);
CREATE TABLE waypoint_node
(
    id               integer primary key autoincrement,
    map_id           integer,
    point_list       text,
    frame_id         text,
    create_timestamp int,
    edit_timestamp   int,
    creator_id       int
);
CREATE TABLE forbidden
(
    id               integer primary key autoincrement,
    map_id           integer,
    point_list       text,
    frame_id         text,
    create_timestamp int,
    edit_timestamp   int,
    creator_id       int
);
CREATE TABLE current_map
(
    id        integer,
    file_path text,
    name      text
);

CREATE TABLE dock_poses (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    map_id integer UNIQUE NOT NULL,
    position_x REAL NOT NULL,
    position_y REAL NOT NULL,
    position_z REAL NOT NULL,
    orientation_x REAL NOT NULL,
    orientation_y REAL NOT NULL,
    orientation_z REAL NOT NULL,
    orientation_w REAL NOT NULL,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
);


DELETE
FROM sqlite_sequence;
COMMIT;
