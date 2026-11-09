# Source- https://youtu.be/Cz3WcZLRaWc
# Project here- recreate airbnb's db
# Db (more storage and secure) vs Spreadsheet apps
# SQL vs NoSQL
# CRUD operations- Create, Read, Update, Delete; Model & Join relational data here
# History- 
    # Relational model of data for banks to reduce redundancy & duplication... (prior is they use network-based [CODASYL?] or hierarchical model)
    # SQL- Structured Query Language
    # ORM- Object Relational Mapping, libraries/tools for working with relational dbs using OOP (e.g. JS:Sequelize, Python:SQLAlchemy, Ruby on Rails:Active Record)
# RDBMS- Relational Database Management System, comprised of- 
    # Db itself- collection of tables and each table has actual data organized in columns & rows
    # Language to manipulate data in db, mostly a variation of SQL (Flavors?- like dialect, e.g. MySQL, Postgre, MS SQLServer (T-SQL?), SQLite; Cockroachdb- for scaling...)

# drawsql.app- to visualize & model data relationships
    # Db Schema- like blueprint that defines the rules & constraints w/in db
        # Boxes:Table- inside is a dictionary where (Table names- lowercase, underscores, plural; more tables = better?)- 
            # Key:Column/Field- can have special constraints (e.g. ENUM, UNIQUE [each value in that column needs to be unique], NOT NULL [each value can't be null], 
            ## PRIMARY KEY [every? table has an "id" w/ the primary key- can't be null and must be unique, ensures that every row can be identified, 
            ## essential for bldg relationships bec. that unique id can then be saved on another table to represent a 1:1 or 1:many relationship {e.g. user can have many rooms, room can have many reviews?}])
            ## Field names- lowercase, underscores, singular, diff. from table and other field names
            # Value:Dtype- can vary bet. diff. sql dbs, many dtypes (correct type optimizes db size & perf); e.g. MySQL- INT, FLOAT, VARCHAR (small strings), TEXT (longer strings)
            ## *Rows/records

            # Normalization?- very little data duplication bet. tables, every table is a unique entity (e.g. Rooms table doesn't need to store any info abt. User [table], just a reference to the user id)
            ## Join later...
        # Lines:Relationships- e.g. 1:1, 1:many

# MySQL installation- https://dev.mysql.com/downloads/installer/
# Open MySQL 8.0 Command Line Client app- not very developer-friendly to just use Command Line, so use SQLTools extension for VSCode (to store connection details, visualize db, view history of queries)
    # mysql -u <uname> -p, then enter pass
# SQLTools- search for extensions > @tag:sqltools-driver (driver for MySQL), then add new connection, test connection, save connection, connect now

# See test.session.sql, bottom- other concepts
# Q- MySQL is relational db that uses tables, some socmed platforms use it but they involve data like photo, video, big files
    # BLOB (Binary Large Object)- a column dtype?..., inefficient for huge platforms (slow, heavy)
    # Store metadata (e.g. user id, filename/url, upload timestamp, size&format) + filepath instead, actual photo/video is saved in a file storage system (e.g. server filesystem [for small apps], cloud storage, CDNs [worldwide])