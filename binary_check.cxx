#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCommand.h"
#include "itkSimpleFilterWatcher.h"

#include "itkBinaryAndImageFilter.h"
#include "itkBinaryOrImageFilter.h"
#include "itkBinaryXorImageFilter.h"
#include "itkBinaryNotImageFilter.h"


int main(int argc, char * argv[])
{

  if( argc != 9 )
    {
    std::cerr << "usage: " << argv[0] << " input1 input2 and or xor not foreground background" << std::endl;
    // std::cerr << "  : " << std::endl;
    exit(1);
    }

  const int dim = 2;

  typedef unsigned char PType;
  typedef itk::Image< PType, dim > IType;

  typedef itk::ImageFileReader< IType > ReaderType;
  ReaderType::Pointer reader1 = ReaderType::New();
  reader1->SetFileName( argv[1] );

  ReaderType::Pointer reader2 = ReaderType::New();
  reader2->SetFileName( argv[2] );

  typedef itk::BinaryAndImageFilter< IType > AndType;
  AndType::Pointer And = AndType::New();
  And->SetInput( 0, reader1->GetOutput() );
  And->SetInput( 1, reader2->GetOutput() );
  And->SetForegroundValue( atoi( argv[7] ) );
  And->SetBackgroundValue( atoi( argv[8] ) );
  itk::SimpleFilterWatcher andWatcher( And, "and" );

  typedef itk::BinaryOrImageFilter< IType > OrType;
  OrType::Pointer Or = OrType::New();
  Or->SetInput( 0, reader1->GetOutput() );
  Or->SetInput( 1, reader2->GetOutput() );
  Or->SetForegroundValue( atoi( argv[7] ) );
  Or->SetBackgroundValue( atoi( argv[8] ) );
  itk::SimpleFilterWatcher orWatcher( Or, "or" );

  typedef itk::BinaryXorImageFilter< IType > XorType;
  XorType::Pointer Xor = XorType::New();
  Xor->SetInput( 0, reader1->GetOutput() );
  Xor->SetInput( 1, reader2->GetOutput() );
  Xor->SetForegroundValue( atoi( argv[7] ) );
  Xor->SetBackgroundValue( atoi( argv[8] ) );
  itk::SimpleFilterWatcher xorWatcher( Xor, "xor" );

  typedef itk::BinaryNotImageFilter< IType > NotType;
  NotType::Pointer Not = NotType::New();
  Not->SetInput( 0, reader1->GetOutput() );
  Not->SetForegroundValue( atoi( argv[7] ) );
  Not->SetBackgroundValue( atoi( argv[8] ) );
  itk::SimpleFilterWatcher notWatcher( Not, "not" );

  typedef itk::ImageFileWriter< IType > WriterType;
  WriterType::Pointer writer = WriterType::New();

  writer->SetInput( And->GetOutput() );
  writer->SetFileName( argv[3] );
  writer->Update();

  writer->SetInput( Or->GetOutput() );
  writer->SetFileName( argv[4] );
  writer->Update();

  writer->SetInput( Xor->GetOutput() );
  writer->SetFileName( argv[5] );
  writer->Update();

  writer->SetInput( Not->GetOutput() );
  writer->SetFileName( argv[6] );
  writer->Update();

  return 0;
}
