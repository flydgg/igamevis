#version 330 core

#extension GL_ARB_shading_language_420pack : enable
#extension GL_ARB_separate_shader_objects : enable

layout(std140, binding = 0) uniform CameraDataBlock {
    vec3 viewPos;
    int isOrtho;
    vec4 orthoBounds;
    float zNear;
    float zFar;
    mat4 view;
    mat4 proj;
    mat4 proj_view;
} cameraData;

layout(std140, binding = 1) uniform ObjectDataBlock {
    float transparent;
    mat4 model;
    mat4 normal;
    vec4 sphereBounds;
} objectData;

layout(std140, binding = 2) uniform UniformBufferObjectBlock {
    int useColor;
    int useNormalSmooth;
} ubo;

uniform int colorMode;
uniform vec3 inputColor = vec3(1.0);
uniform int uUseLighting = 1;

layout(location = 0) in vec3 in_MCPosition;
layout(location = 1) in vec3 in_VCPosition;
layout(location = 2) in vec4 in_Color;
layout(location = 3) in vec3 in_Normal;
layout(location = 4) in vec2 in_UV;

layout(location = 0) out vec4 out_ScreenColor;

vec3 BlinnPhong(vec3 n, vec3 fragPos) {
    vec3 lightDir = normalize(vec3(0.0, 0.0, 1.0));
    float diffuse = max(dot(lightDir, n), 0.0);
    vec3 viewDir = normalize(cameraData.viewPos - fragPos);
    vec3 halfDir = normalize(lightDir + viewDir);
    float specular = diffuse > 0.0 ? pow(max(dot(n, halfDir), 0.0), 32.0) : 0.0;
    return vec3(0.4) + vec3(diffuse * 0.5 + specular * 0.5);
}

void main() {
    vec3 baseColor = inputColor;
    if (colorMode == 1 || (colorMode == 0 && ubo.useColor == 1)) {
        baseColor = in_Color.rgb;
    }

    vec3 color = baseColor;
    if (colorMode == 0 && uUseLighting == 1) {
        vec3 n;
        if (ubo.useNormalSmooth == 1) {
            n = normalize(in_Normal);
        } else {
            vec3 dx = dFdx(in_VCPosition);
            vec3 dy = dFdy(in_VCPosition);
            n = normalize(cross(dx, dy));
        }
        if ((cameraData.isOrtho == 1 && n.z < 0.0) ||
            (cameraData.isOrtho == 0 && dot(n, in_VCPosition) > 0.0)) {
            n = -n;
        }
        color *= BlinnPhong(n, in_MCPosition);
    }

    float alpha = colorMode == 2 ? objectData.transparent
                                 : in_Color.a * objectData.transparent;
    out_ScreenColor = vec4(color, alpha);
}
