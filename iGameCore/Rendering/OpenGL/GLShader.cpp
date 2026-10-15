//
// Created by Sumzeek on 12/9/2024.
//
#include "GLShader.h"
#include <cctype>
#include <filesystem>
#include <vector>

IGAME_NAMESPACE_BEGIN

GLShader::GLShader() {}

GLShader::~GLShader() {
    if (m_Handle != 0) {
        glDeleteShader(m_Handle);
        m_Handle = 0;
    }
}

SmartPointer<GLShader> GLShader::CreateShader(const std::string& path,
                                              GLenum shaderType) {
    auto shader = GLShader::New();
    shader->SetName(std::filesystem::path(path).filename().string());
    shader->Compile(path.c_str(), shaderType);
    return shader;
}

void GLShader::Compile(const char* const file_path, GLenum type) {
    m_Handle = glCreateShader(type);

    std::string sourceStr = ReadFile(file_path);
#ifdef IGAME_PLATFORM_MACOS
    // Apple's OpenGL 4.1 driver does not expose ARB_shading_language_420pack.
    // Uniform-block bindings are assigned with glUniformBlockBinding instead.
    const auto removeLine = [&sourceStr](const char* line) {
        size_t position = 0;
        while ((position = sourceStr.find(line, position)) != std::string::npos) {
            const size_t lineEnd = sourceStr.find('\n', position);
            sourceStr.erase(position,
                            lineEnd == std::string::npos
                                    ? std::string::npos
                                    : lineEnd - position + 1);
        }
    };
    removeLine("#extension GL_ARB_shading_language_420pack : enable");
    removeLine("#extension GL_ARB_separate_shader_objects : enable");

    const std::string bindingQualifier = ", binding = ";
    size_t bindingPosition = 0;
    while ((bindingPosition = sourceStr.find(bindingQualifier,
                                             bindingPosition)) !=
           std::string::npos) {
        const size_t qualifierEnd = sourceStr.find(')', bindingPosition);
        if (qualifierEnd == std::string::npos) { break; }
        // Keep the rest of the layout qualifiers (for example std140), and
        // remove only the 4.2 binding assignment.
        sourceStr.erase(bindingPosition, qualifierEnd - bindingPosition);
    }

    // GLSL 4.10 on Apple's driver accepts locations for vertex attributes and
    // fragment outputs, but not for inter-stage varyings. Match those varyings
    // by name, as GLSL 3.30 requires, and strip the unsupported qualifiers.
    std::vector<std::pair<std::string, std::string>> varyingRenames;
    size_t locationPosition = 0;
    while ((locationPosition = sourceStr.find("layout", locationPosition)) !=
           std::string::npos) {
        const size_t openParen = sourceStr.find('(', locationPosition + 6);
        const size_t closeParen = sourceStr.find(')', openParen);
        if (openParen == std::string::npos || closeParen == std::string::npos) {
            break;
        }

        const std::string layoutContents =
                sourceStr.substr(openParen + 1, closeParen - openParen - 1);
        if (layoutContents.find("location") == std::string::npos) {
            locationPosition = closeParen + 1;
            continue;
        }

        size_t declaration = closeParen + 1;
        while (declaration < sourceStr.size() &&
               std::isspace(static_cast<unsigned char>(sourceStr[declaration]))) {
            ++declaration;
        }
        const bool isInput = sourceStr.compare(declaration, 2, "in") == 0 &&
                             (declaration + 2 >= sourceStr.size() ||
                              !std::isalnum(static_cast<unsigned char>(
                                      sourceStr[declaration + 2])));
        const bool isOutput = sourceStr.compare(declaration, 3, "out") == 0 &&
                              (declaration + 3 >= sourceStr.size() ||
                               !std::isalnum(static_cast<unsigned char>(
                                       sourceStr[declaration + 3])));
        const bool removeLocation =
                (type == GL_GEOMETRY_SHADER && (isInput || isOutput)) ||
                (type == GL_VERTEX_SHADER && isOutput) ||
                (type == GL_FRAGMENT_SHADER && isInput);
        if (removeLocation) {
            const size_t equalsPosition = layoutContents.find('=');
            int location = 0;
            if (equalsPosition != std::string::npos) {
                try {
                    location = std::stoi(layoutContents.substr(equalsPosition + 1));
                } catch (...) {
                    location = 0;
                }
            }

            size_t identifierStart = declaration + (isInput ? 2 : 3);
            while (identifierStart < sourceStr.size() &&
                   std::isspace(static_cast<unsigned char>(
                           sourceStr[identifierStart]))) {
                ++identifierStart;
            }
            while (identifierStart < sourceStr.size() &&
                   !std::isspace(static_cast<unsigned char>(
                           sourceStr[identifierStart]))) {
                ++identifierStart;
            }
            while (identifierStart < sourceStr.size() &&
                   std::isspace(static_cast<unsigned char>(
                           sourceStr[identifierStart]))) {
                ++identifierStart;
            }
            const size_t identifierEnd = sourceStr.find_first_not_of(
                    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_",
                    identifierStart);
            if (identifierStart < sourceStr.size()) {
                const size_t end = identifierEnd == std::string::npos
                                           ? sourceStr.size()
                                           : identifierEnd;
                varyingRenames.emplace_back(
                        sourceStr.substr(identifierStart, end - identifierStart),
                        "igame_varying_" + std::to_string(location));
            }
            sourceStr.erase(locationPosition, declaration - locationPosition);
            continue;
        }
        locationPosition = declaration;
    }

    for (const auto& [name, replacement] : varyingRenames) {
        size_t position = 0;
        while ((position = sourceStr.find(name, position)) !=
               std::string::npos) {
            const bool leftBoundary =
                    position == 0 ||
                    (!std::isalnum(static_cast<unsigned char>(
                             sourceStr[position - 1])) &&
                     sourceStr[position - 1] != '_');
            const size_t afterName = position + name.size();
            const bool rightBoundary =
                    afterName == sourceStr.size() ||
                    (!std::isalnum(static_cast<unsigned char>(
                             sourceStr[afterName])) &&
                     sourceStr[afterName] != '_');
            if (leftBoundary && rightBoundary) {
                sourceStr.replace(position, name.size(), replacement);
                position += replacement.size();
            } else {
                position = afterName;
            }
        }
    }
#endif
    const char* source = sourceStr.c_str();

    auto src_cpy = source;
    glShaderSource(m_Handle, 1, &src_cpy, 0);
    glCompileShader(m_Handle);

    CheckCompileErrors();
}

void GLShader::CheckCompileErrors() {
    int success;
    std::string infoLog;
    infoLog.resize(BUFSIZ);

    glGetShaderiv(m_Handle, GL_COMPILE_STATUS, &success);

    if (!success) {
        glGetShaderInfoLog(m_Handle, BUFSIZ, NULL, infoLog.data());
        IGAME_RENDERING_ERROR("[GLShader::CheckCompileErrors] Shader Name: '{}', "
                "Error: SHADER_COMPILATION_FAILED, Details: {}",
                this->GetName(), infoLog);
    }
}

std::string GLShader::ReadFile(const char* file_path) {
    std::ifstream file(file_path, std::ios::in | std::ios::binary);
    if (file) {
        std::string contents;
        file.seekg(0, std::ios::end);
        contents.resize(file.tellg());
        file.seekg(0, std::ios::beg);
        file.read(contents.data(), contents.size());
        file.close();
        return contents;
    } else {
        IGAME_RENDERING_ERROR("[GLShader::ReadFile] Failed to open file '{}', "
                              "Shader Name: '{}'",
                              file_path, this->GetName());
        return std::string();
    }
}

GLUniform::GLUniform(){};

GLUniform::~GLUniform(){};

unsigned int GLUniform::Index() const { return m_Index; }

GLShaderProgram::GLShaderProgram() {}

GLShaderProgram::~GLShaderProgram() {
    if (m_Handle != 0) {
        glDeleteProgram(m_Handle);
        m_Handle = 0;
    }
}

void GLShaderProgram::Use() const { glUseProgram(m_Handle); }

GLuint GLShaderProgram::ProgramID() const { return m_Handle; }

// SetUniform1
void GLShaderProgram::SetUniformi(const char* name, int value) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform1i(uniform->Index(), value);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform1i(m_Handle, uniform->Index(), value);
#endif
}

void GLShaderProgram::SetUniformf(const char* name, float value) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform1f(uniform->Index(), value);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform1f(m_Handle, uniform->Index(), value);
#endif
}

void GLShaderProgram::SetUniformui(const char* name, unsigned int value) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform1ui(uniform->Index(), value);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform1ui(m_Handle, uniform->Index(), value);
#endif
}

// SetUniform2
void GLShaderProgram::SetUniform2i(const char* name,
                                   const igm::ivec2& vec2) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform2i(uniform->Index(), vec2.x, vec2.y);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform2iv(m_Handle, uniform->Index(), 1, vec2.data());
#endif
}

void GLShaderProgram::SetUniform2f(const char* name,
                                   const igm::vec2& vec2) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform2f(uniform->Index(), vec2.x, vec2.y);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform2fv(m_Handle, uniform->Index(), 1, vec2.data());
#endif
}

void GLShaderProgram::SetUniform2ui(const char* name,
                                    const igm::uvec2& vec2) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform2ui(uniform->Index(), vec2.x, vec2.y);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform2uiv(m_Handle, uniform->Index(), 1, vec2.data());
#endif
}

// SetUniform3
void GLShaderProgram::SetUniform3i(const char* name,
                                   const igm::ivec3& vec3) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform3i(uniform->Index(), vec3.x, vec3.y, vec3.z);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform3iv(m_Handle, uniform->Index(), 1, vec3.data());
#endif
}

void GLShaderProgram::SetUniform3f(const char* name,
                                   const igm::vec3& vec3) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform3f(uniform->Index(), vec3.x, vec3.y, vec3.z);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform3fv(m_Handle, uniform->Index(), 1, vec3.data());
#endif
}

void GLShaderProgram::SetUniform3ui(const char* name,
                                    const igm::uvec3& vec3) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform3ui(uniform->Index(), vec3.x, vec3.y, vec3.z);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform3uiv(m_Handle, uniform->Index(), 1, vec3.data());
#endif
}

// SetUniform4i
void GLShaderProgram::SetUniform4i(const char* name,
                                   const igm::ivec4& vec4) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform4i(uniform->Index(), vec4.x, vec4.y, vec4.z, vec4.w);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform4iv(m_Handle, uniform->Index(), 1, vec4.data());
#endif
}

void GLShaderProgram::SetUniform4f(const char* name,
                                   const igm::vec4& vec4) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform4f(uniform->Index(), vec4.x, vec4.y, vec4.z, vec4.w);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform4fv(m_Handle, uniform->Index(), 1, vec4.data());
#endif
}

void GLShaderProgram::SetUniform4ui(const char* name,
                                    const igm::uvec4& vec4) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniform4ui(uniform->Index(), vec4.x, vec4.y, vec4.z, vec4.w);
#elif IGAME_OPENGL_VERSION_460
    glProgramUniform4uiv(m_Handle, uniform->Index(), 1, vec4.data());
#endif
}

// SetUniformMatrix
void GLShaderProgram::SetUniformMatrix3x3(const char* name,
                                          const igm::mat3& mat3) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniformMatrix3fv(uniform->Index(), 1, GL_FALSE, mat3.data());
#elif IGAME_OPENGL_VERSION_460
    glProgramUniformMatrix3fv(m_Handle, uniform->Index(), 1, false,
                              mat3.data());
#endif
}

void GLShaderProgram::SetUniformMatrix4x4(const char* name,
                                          const igm::mat4& mat4) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniformMatrix4fv(uniform->Index(), 1, GL_FALSE, mat4.data());
#elif IGAME_OPENGL_VERSION_460
    glProgramUniformMatrix4fv(m_Handle, uniform->Index(), 1, false,
                              mat4.data());
#endif
}

void GLShaderProgram::SetUniformMatrix4x4(const char* name, bool transpose,
                                          const igm::mat4& mat4) const {
    SmartPointer<GLUniform> uniform = GetUniformLocation(name);
    if (uniform == nullptr) { return; }

#ifdef IGAME_OPENGL_VERSION_330
    glUniformMatrix4fv(uniform->Index(), 1, transpose ? GL_TRUE : GL_FALSE,
                       mat4.data());
#elif IGAME_OPENGL_VERSION_460
    glProgramUniformMatrix4fv(m_Handle, uniform->Index(), 1,
                              transpose ? GL_TRUE : GL_FALSE, mat4.data());
#endif
}

void GLShaderProgram::MapUniformBlock(const char* uniformBlockName,
                                      uint32_t uniformBlockBinding,
                                      SmartPointer<GLBuffer> m_UBOBlock) {
#ifdef __EMSCRIPTEN__
    (void) uniformBlockName;
    (void) uniformBlockBinding;
    (void) m_UBOBlock;
    return;
#endif
    GLuint blockIndex = glGetUniformBlockIndex(m_Handle, uniformBlockName);
    if (blockIndex == GL_INVALID_INDEX) {
        IGAME_RENDERING_ERROR("[GLShaderProgram::MapUniformBlock] Shader '{}' does "
                "not contain the Uniform Block '{}'.",
                this->GetName(), uniformBlockName);
        return;
    }
    glUniformBlockBinding(m_Handle, blockIndex, uniformBlockBinding);
    m_UBOBlock->Target(GL_UNIFORM_BUFFER);
    m_UBOBlock->BindBase(uniformBlockBinding);
}


GLVertexAttribute GLShaderProgram::GetAttribLocation(const char* name) {
    int location = glGetAttribLocation(m_Handle, name);
    if (location == -1) {
        IGAME_RENDERING_ERROR("[GLShaderProgram::GetAttribLocation] Shader '{}' "
                "does not contain the attribute '{}' (location: -1).",
                this->GetName(), name);
    }

    return GLVertexAttribute{static_cast<unsigned int>(location)};
}

SmartPointer<GLUniform>
GLShaderProgram::GetUniformLocation(const char* name) const {
    int location = glGetUniformLocation(m_Handle, name);
    if (location == -1) {
#ifdef __EMSCRIPTEN__
        return nullptr;
#else
        IGAME_RENDERING_ERROR("[GLShaderProgram::GetUniformLocation] Shader '{}' "
                "does not contain the uniform '{}' (location: -1).",
                this->GetName(), name);
#endif
    }

    SmartPointer<GLUniform> uniform = GLUniform::New();
    uniform->m_Index = static_cast<unsigned int>(location);
    return uniform;
}

void GLShaderProgram::CheckCompileErrors() {
    int success;
    std::string infoLog;
    infoLog.resize(BUFSIZ);

    glGetProgramiv(m_Handle, GL_LINK_STATUS, &success);

    if (!success) {
        glGetProgramInfoLog(m_Handle, BUFSIZ, NULL, infoLog.data());
        IGAME_RENDERING_ERROR("[GLShaderProgram::CheckCompileErrors] Shader program "
                "'{}' linkage failed. Error: {}",
                this->GetName(), infoLog);
    }
#ifdef __EMSCRIPTEN__
    else {
        glValidateProgram(m_Handle);
        GLint validated = GL_FALSE;
        glGetProgramiv(m_Handle, GL_VALIDATE_STATUS, &validated);
        if (validated != GL_TRUE) {
            glGetProgramInfoLog(m_Handle, BUFSIZ, NULL, infoLog.data());
            IGAME_RENDERING_ERROR(
                    "[GLShaderProgram::CheckCompileErrors] Shader program "
                    "'{}' validation failed after link. id={}, info={}",
                    this->GetName(), m_Handle, infoLog);
        } else {
            IGAME_RENDERING_INFO(
                    "[GLShaderProgram::CheckCompileErrors] Shader program "
                    "'{}' linked and validated. id={}",
                    this->GetName(), m_Handle);
        }
    }
#endif
}

IGAME_NAMESPACE_END
