using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NPOI.HSSF.UserModel;
using NPOI.SS.UserModel;
using System.IO;
using System.Data;
using NPOI.XSSF.UserModel;
using NPOI.HSSF.Util;
using NPOI.SS.Util;

namespace UAV_WPF.Tools
{
    public class NPOIMethod
    {
        public void ReadFile(string path)
        {
            HSSFWorkbook hssfwb;
            using (FileStream file = new FileStream(path, FileMode.Open, FileAccess.Read))
            {
                hssfwb = new HSSFWorkbook(file);
            }

            ISheet sheet = hssfwb.GetSheet("Arkusz1");
            for (int row = 0; row <= sheet.LastRowNum; row++)
            {
                if (sheet.GetRow(row) != null) //null is when the row only contains empty cells 
                {
                    string str = sheet.GetRow(row).GetCell(0).StringCellValue;
                }
            }
        }
        /// <summary>
        /// 从一个excel表中读数据到datatable
        /// </summary>
        /// <param name="pRutaArchivo">文件名</param>
        /// <param name="pHojaIndex">工作表</param>
        /// <returns></returns>
        public static List<WayPointInfo> Excel_To_List(string pRutaArchivo, int pHojaIndex)
        {
            pRutaArchivo = AppDomain.CurrentDomain.BaseDirectory + @"newbook.xlsx";
            List<WayPointInfo> wayPointInfos = new List<WayPointInfo>();
            WayPointInfo wayPointInfo;
            try
            {
                if (System.IO.File.Exists(pRutaArchivo))
                {

                    IWorkbook workbook = null;  //IWorkbook determina si es xls o xlsx              
                    ISheet worksheet = null;
                    string first_sheet_name = "";

                    using (FileStream FS = new FileStream(pRutaArchivo, FileMode.Open, FileAccess.Read))
                    {
                        workbook = WorkbookFactory.Create(FS);          //Abre tanto XLS como XLSX
                        worksheet = workbook.GetSheetAt(pHojaIndex);    //Obtener Hoja por indice
                        first_sheet_name = worksheet.SheetName;         //Obtener el nombre de la Hoja
                        // Leer Fila por fila desde la primera
                        for (int rowIndex = 0; rowIndex <= worksheet.LastRowNum; rowIndex++)
                        {

                            wayPointInfo = new WayPointInfo();
                            IRow row = worksheet.GetRow(rowIndex);
                            IRow row2 = null;
                            IRow row3 = null;

                            if (rowIndex == 0)
                            {
                                row2 = worksheet.GetRow(rowIndex + 1); //Si es la Primera fila, obtengo tambien la segunda para saber el tipo de datos
                                row3 = worksheet.GetRow(rowIndex + 2); //Y la tercera tambien por las dudas
                            }

                            if (row != null) //null is when the row only contains empty cells 
                            {
                            
                                    wayPointInfo = new WayPointInfo();
          
                                wayPointInfo.command = row.Cells[0].StringCellValue.ToString();
                                wayPointInfo.lat = row.Cells[1].NumericCellValue;
                                wayPointInfo.lng = row.Cells[2].NumericCellValue;
                                wayPointInfo.heigh = row.Cells[3].NumericCellValue;
                                wayPointInfo.stayTime = Convert.ToDateTime(row.Cells[4].StringCellValue);
                                wayPointInfos.Add(wayPointInfo);
                                #region
                                //Leer cada Columna de la fila
                                /*          foreach (ICell cell in row.Cells)
                                          {
                                              object valorCell = null;
                                              string cellType = "";
                                              string[] cellType2 = new string[2];

                                              if (rowIndex == 0) //Asumo que la primera fila contiene los titlos:
                                              {
                                                  for (int i = 0; i < 2; i++)
                                                  {
                                                      ICell cell2 = null;
                                                      if (i == 0)
                                                      {
                                                          cell2 = row2.GetCell(cell.ColumnIndex);
                                                      }
                                                      else
                                                      {
                                                          cell2 = row3.GetCell(cell.ColumnIndex);
                                                      }

                                                      if (cell2 != null)
                                                      {
                                                          switch (cell2.CellType)
                                                          {
                                                              case CellType.Blank: break;
                                                              case CellType.Boolean: cellType2[i] = "System.Boolean"; break;
                                                              case CellType.String: cellType2[i] = "System.String"; break;
                                                              case CellType.Numeric:
                                                                  if (HSSFDateUtil.IsCellDateFormatted(cell2)) { cellType2[i] = "System.DateTime"; }
                                                                  else
                                                                  {
                                                                      cellType2[i] = "System.Double";  //valorCell = cell2.NumericCellValue;
                                                                  }
                                                                  break;

                                                              case CellType.Formula:
                                                                  bool continuar = true;
                                                                  switch (cell2.CachedFormulaResultType)
                                                                  {
                                                                      case CellType.Boolean: cellType2[i] = "System.Boolean"; break;
                                                                      case CellType.String: cellType2[i] = "System.String"; break;
                                                                      case CellType.Numeric:
                                                                          if (HSSFDateUtil.IsCellDateFormatted(cell2)) { cellType2[i] = "System.DateTime"; }
                                                                          else
                                                                          {
                                                                              try
                                                                              {
                                                                                  //DETERMINAR SI ES BOOLEANO
                                                                                  if (cell2.CellFormula == "TRUE()") { cellType2[i] = "System.Boolean"; continuar = false; }
                                                                                  if (continuar && cell2.CellFormula == "FALSE()") { cellType2[i] = "System.Boolean"; continuar = false; }
                                                                                  if (continuar) { cellType2[i] = "System.Double"; continuar = false; }
                                                                              }
                                                                              catch { }
                                                                          }
                                                                          break;
                                                                  }
                                                                  break;
                                                              default:
                                                                  cellType2[i] = "System.String"; break;
                                                          }
                                                      }
                                                  }

                                                  //Resolver las diferencias de Tipos
                                                  if (cellType2[0] == cellType2[1]) { cellType = cellType2[0]; }
                                                  else
                                                  {
                                                      if (cellType2[0] == null) cellType = cellType2[1];
                                                      if (cellType2[1] == null) cellType = cellType2[0];
                                                      if (cellType == "") cellType = "System.String";
                                                  }

                                                  //Obtener el nombre de la Columna
                                                  string colName = "Column_{0}";
                                                  try
                                                  {
                                                      colName = cell.StringCellValue;
                                                  }
                                                  catch
                                                  {
                                                      colName = string.Format(colName, colIndex);
                                                  }

                                                  //Verificar que NO se repita el Nombre de la Columna     
                                              }
                                              else
                                              {
                                                  //Las demas filas son registros:
                                                  switch (cell.CellType)
                                                  {
                                                      case CellType.Blank: valorCell = DBNull.Value; break;
                                                      case CellType.Boolean: valorCell = cell.BooleanCellValue; break;
                                                      case CellType.String: valorCell = cell.StringCellValue; break;
                                                      case CellType.Numeric:
                                                          if (HSSFDateUtil.IsCellDateFormatted(cell))
                                                          {
                                                              valorCell = cell.DateCellValue;
                                                          }
                                                          else
                                                          {
                                                              valorCell = cell.NumericCellValue;
                                                          }
                                                          break;
                                                      case CellType.Formula:
                                                          switch (cell.CachedFormulaResultType)
                                                          {
                                                              case CellType.Blank: valorCell = DBNull.Value; break;
                                                              case CellType.String: valorCell = cell.StringCellValue; break;
                                                              case CellType.Boolean: valorCell = cell.BooleanCellValue; break;
                                                              case CellType.Numeric:
                                                                  if (HSSFDateUtil.IsCellDateFormatted(cell))
                                                                  {
                                                                      valorCell = cell.DateCellValue;
                                                                  }
                                                                  else
                                                                  {
                                                                      valorCell = cell.NumericCellValue;
                                                                  }
                                                                  break;
                                                          }
                                                          break;
                                                      default: valorCell = cell.StringCellValue; break;
                                                  }
                                                  //Agregar el nuevo Registro

                                              }
                                          }
                                      }

                                  }

                              }
                          }
                          else
                          {
                              throw new Exception("ERROR 404: El archivo especificado NO existe.");
                          }
                      return Tabla;*/
                                #endregion
                            }
                        }
                    }  
                }
            }
            catch(Exception ex)
            {
                throw ex;
            }
            return wayPointInfos;
        }
        public static void CreateExcel()
        {
            var newFile = @"newbook.xlsx";
            if (File.Exists(newFile))
                File.Delete(newFile);

            using (var fs = new FileStream(newFile, FileMode.Create, FileAccess.Write))
            {

                IWorkbook workbook = new XSSFWorkbook();

                ISheet sheet1 = workbook.CreateSheet("Sheet1");
                //开始行，结束行，开始列，结束列
                sheet1.AddMergedRegion(new CellRangeAddress(0, 0, 0, 10));
                var rowIndex = 0;
                IRow row = sheet1.CreateRow(rowIndex);
                row.Height = 30 * 80;
                row.CreateCell(0).SetCellValue("this is content");
                sheet1.AutoSizeColumn(0);
                rowIndex++;
                //创建工作表
                var sheet2 = workbook.CreateSheet("Sheet2");
                //创建工作表的样式
                var style1 = workbook.CreateCellStyle();
                //设置样式属性
                style1.FillForegroundColor = HSSFColor.Blue.Index2;
                style1.FillPattern = FillPattern.SolidForeground;

                var style2 = workbook.CreateCellStyle();
                style2.FillForegroundColor = HSSFColor.Yellow.Index2;
                style2.FillPattern = FillPattern.SolidForeground;
                //创建行的第n行和第n列
                var cell2 = sheet2.CreateRow(0).CreateCell(0);
                cell2.CellStyle = style1;
                //设置行的值
                cell2.SetCellValue(0);
                try
                {
                    var rows = sheet2.CreateRow(1);
                    rows.CreateCell(2).SetCellValue("sss");
                    rows.CreateCell(3).SetCellValue("saa");
                    rows.CreateCell(4).SetCellValue("dddd");
                    cell2.CellStyle = style2;
                }
                catch(Exception ex)
                {
                    throw new Exception(ex.ToString());
                }
               
              
                
                workbook.Write(fs);
            }
        }
    }
}
