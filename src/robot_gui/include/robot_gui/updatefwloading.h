#ifndef UPDATEFWLOADING_H
#define UPDATEFWLOADING_H

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QMovie>
#include <QVBoxLayout>

namespace Ui {
class updateFWLoading;
}

class updateFWLoading : public QDialog
{
  Q_OBJECT

public:
  explicit updateFWLoading(QWidget *parent = nullptr);
  ~updateFWLoading();

private:
  Ui::updateFWLoading *ui;
  QLabel *loadingLabel;
  QMovie *loadingMovie;
};

#endif // UPDATEFWLOADING_H
