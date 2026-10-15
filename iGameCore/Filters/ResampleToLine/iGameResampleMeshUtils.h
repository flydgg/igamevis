#pragma once

#include "iGameUnstructuredMesh.h"

IGAME_NAMESPACE_BEGIN

/** Flatten a supported mesh or composite mesh into one unstructured probe mesh. */
UnstructuredMesh::Pointer BuildResampleProbeMesh(DataObject* object, int* blockCount = nullptr);

IGAME_NAMESPACE_END
