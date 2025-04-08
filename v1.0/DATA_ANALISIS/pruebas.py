import cadquery as cq

# Parámetros del cohete
outer_diameter = 50.0  # mm
shoulder_diameter = outer_diameter - 0.5  # mm (saliente ligeramente más pequeño)
shoulder_height = 30.0  # mm
body_height = 60.0  # mm
wall_thickness = 2.0  # mm

# Cuerpo del cohete (cilindro con saliente interior)
body = (
    cq.Workplane("XY")
    .circle(outer_diameter / 2)
    .circle((outer_diameter - wall_thickness * 2) / 2)
    .extrude(body_height)
)

# Saliente interior en la parte superior
shoulder = (
    cq.Workplane("XY")
    .workplane(offset=body_height)
    .circle(shoulder_diameter / 2)
    .circle((shoulder_diameter - wall_thickness * 2) / 2)
    .extrude(-shoulder_height)
)

# Unimos ambas partes
rocket_body = body.union(shoulder)

# Exportamos como STL
file_path = "rocket_body_with_shoulder.stl"
cq.exporters.export(rocket_body, file_path)