using System;
using System.IO;

class Program
{
    static void Main(string[] args)
    {
        // 指定包含文件夹路径的文件路径
        string pathsFilePath = @"c:\Users\15014\Desktop\deep_learn\folder_list";

        try
        {
            // 读取文件中的所有行，每行代表一个文件夹路径
            string[] folderPaths = File.ReadAllLines(pathsFilePath);

            foreach (string folderPath in folderPaths)
            {
                // 输出正在检查的文件夹路径
                Console.WriteLine("正在读取文件夹: " + folderPath);

                // 检查文件夹是否存在
                if (Directory.Exists(folderPath))
                {
                    // 获取文件夹中的所有文件
                    string[] files = Directory.GetFiles(folderPath);

                    // 输出文件数量以确认是否有文件被遍历到
                    Console.WriteLine("文件夹中文件总数: " + files.Length);

                    foreach (string file in files)
                    {
                        // 输出每个文件的路径
                        Console.WriteLine("正在检查文件: " + file);

                        // 检查文件是否为图片格式（这里仅列出.jpg和.png）
                        if (Path.GetExtension(file).ToLower() == ".jpg" || Path.GetExtension(file).ToLower() == ".png")
                        {
                            Console.WriteLine("找到图片文件: " + file);
                            // 你可以在这里添加更多的操作，比如加载图片到内存中进行处理
                        }
                    }
                }
                else
                {
                    Console.WriteLine("文件夹不存在: " + folderPath);
                }
            }
        }
        catch (Exception e)
        {
            Console.WriteLine("读取文件夹时出错: " + e.Message);
        }
    }
}

