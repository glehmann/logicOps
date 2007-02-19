#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCommand.h"
#include "itkSimpleFilterWatcher.h"

#include "itkEqualImageFilter.h"
#include "itkNotEqualImageFilter.h"
#include "itkGreaterEqualImageFilter.h"
#include "itkGreaterImageFilter.h"
#include "itkLessEqualImageFilter.h"
#include "itkLessImageFilter.h"

#include "itkSmoothingRecursiveGaussianImageFilter.h"

int main(int, char * argv[])
{
  const int dim = 2;
  
  typedef unsigned char PType;
  typedef itk::Image< PType, dim > IType;

  typedef itk::ImageFileReader< IType > ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( argv[1] );

  typedef itk::SmoothingRecursiveGaussianImageFilter<IType, IType> SmoothType;
  SmoothType::Pointer smoother = SmoothType::New();

  smoother->SetInput(reader->GetOutput());
  smoother->SetSigma(1);

  typedef itk::EqualImageFilter<IType, IType, IType> EqType;
  typedef itk::NotEqualImageFilter<IType, IType, IType> NEType;
  typedef itk::GreaterImageFilter<IType, IType, IType> GTType;
  typedef itk::GreaterEqualImageFilter<IType, IType, IType> GTEqType;
  typedef itk::LessImageFilter<IType, IType, IType> LTType;
  typedef itk::LessEqualImageFilter<IType, IType, IType> LTEqType;

  EqType::Pointer eq = EqType::New();
  NEType::Pointer ne = NEType::New();
  GTType::Pointer gt = GTType::New();
  GTEqType::Pointer gte = GTEqType::New();
  LTType::Pointer lt = LTType::New();
  LTEqType::Pointer lte = LTEqType::New();


  eq->SetInput(reader->GetOutput());
  ne->SetInput(reader->GetOutput());
  gt->SetInput(reader->GetOutput());
  gte->SetInput(reader->GetOutput());
  lt->SetInput(reader->GetOutput());
  lte->SetInput(reader->GetOutput());

  eq->SetInput2(smoother->GetOutput());
  ne->SetInput2(smoother->GetOutput());
  gt->SetInput2(smoother->GetOutput());
  gte->SetInput2(smoother->GetOutput());
  lt->SetInput2(smoother->GetOutput());
  lte->SetInput2(smoother->GetOutput());

  


  typedef itk::ImageFileWriter< IType > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( eq->GetOutput() );
  writer->SetFileName( argv[2] );
  writer->Update();

  writer->SetInput( gt->GetOutput() );
  writer->SetFileName( argv[3] );
  writer->Update();

  writer->SetInput( gte->GetOutput() );
  writer->SetFileName( argv[4] );
  writer->Update();

  writer->SetInput( lt->GetOutput() );
  writer->SetFileName( argv[5] );
  writer->Update();

  writer->SetInput( lte->GetOutput() );
  writer->SetFileName( argv[6] );
  writer->Update();

  writer->SetInput( smoother->GetOutput() );
  writer->SetFileName( argv[7] );
  writer->Update();

  writer->SetInput( ne->GetOutput() );
  writer->SetFileName( argv[8] );
  writer->Update();

  return 0;
}

