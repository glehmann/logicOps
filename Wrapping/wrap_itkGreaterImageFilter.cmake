WRAP_CLASS("itk::GreaterImageFilter" POINTER_WITH_SUPERCLASS)
  FOREACH(t ${WRAP_ITK_SCALAR})
    WRAP_IMAGE_FILTER_COMBINATIONS("${t}" "${t}" "${WRAP_ITK_INT}")
  ENDFOREACH(t)
END_WRAP_CLASS()
