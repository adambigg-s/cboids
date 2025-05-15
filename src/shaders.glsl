@vs simple_vs
in vec4 position;

void main() {
    gl_Position = vec4(position.x, position.y, position.z, 1.);
}
@end

@fs simple_fs
out vec4 FragColor;

void main() {
    FragColor = vec4(1., 0.5, 0.2, 1.);
}
@end

@program simple simple_vs simple_fs
