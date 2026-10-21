#include "wizard/project_page.h"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace wpiemu::app {

ProjectPage::ProjectPage(WizardViewModel& model, RecentProjects& recent, const UiResources& ui, QWidget* parent)
    : WizardPage(parent),
      model_(model),
      recent_(recent),
      ui_(ui),
      dropZone_(new ProjectDropZone),
      dropIcon_(new QLabel),
      pathEdit_(new QLineEdit),
      browseButton_(new QPushButton(tr("Browse…"))),
      banner_(new Banner(ui)),
      detailsCard_(makeCard()),
      seasonValue_(makeValueLabel()),
      languageValue_(makeValueLabel()),
      teamValue_(makeValueLabel()),
      vendorValue_(makeValueLabel()),
      setupValue_(makeValueLabel()),
      recentTitle_(makeLabel(tr("Recent projects"), "section")),
      recentList_(new QListWidget) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(14);

    // Drop target with the path field inside it.
    auto* dropLayout = new QVBoxLayout(dropZone_);
    dropLayout->setContentsMargins(24, 20, 24, 20);
    dropLayout->setSpacing(10);
    dropIcon_->setAlignment(Qt::AlignCenter);
    auto* dropText = makeLabel(tr("Drop your WPILib project folder here"), "section");
    dropText->setAlignment(Qt::AlignCenter);
    auto* orText = makeLabel(tr("or paste its location"), "muted");
    orText->setAlignment(Qt::AlignCenter);
    auto* pathRow = new QHBoxLayout;
    pathEdit_->setPlaceholderText(tr("C:\\Users\\you\\robot-2026"));
    pathEdit_->setClearButtonEnabled(true);
    pathEdit_->setAccessibleName(tr("Project folder"));
    pathRow->addWidget(pathEdit_, 1);
    pathRow->addWidget(browseButton_);
    dropLayout->addWidget(dropIcon_);
    dropLayout->addWidget(dropText);
    dropLayout->addWidget(orText);
    dropLayout->addLayout(pathRow);
    root->addWidget(dropZone_);

    root->addWidget(banner_);

    auto* grid = new QGridLayout(detailsCard_);
    grid->setContentsMargins(16, 14, 16, 14);
    grid->setHorizontalSpacing(18);
    grid->setVerticalSpacing(8);
    grid->setColumnStretch(1, 1);
    const std::pair<QString, QLabel*> rows[] = {{tr("Season"), seasonValue_},
                                                {tr("Language"), languageValue_},
                                                {tr("Team number"), teamValue_},
                                                {tr("Vendor libraries"), vendorValue_},
                                                {tr("Robot setup"), setupValue_}};
    int row = 0;
    for (const auto& [name, value] : rows) {
        grid->addWidget(makeLabel(name, "muted"), row, 0, Qt::AlignTop);
        grid->addWidget(value, row, 1);
        ++row;
    }
    detailsCard_->hide();
    root->addWidget(detailsCard_);

    root->addWidget(recentTitle_);
    recentList_->setMaximumHeight(150);
    recentList_->setTextElideMode(Qt::ElideMiddle);
    recentList_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    recentList_->setAccessibleName(tr("Recent projects"));
    root->addWidget(recentList_);
    root->addStretch(1);

    typingTimer_.setSingleShot(true);
    typingTimer_.setInterval(300);
    connect(&typingTimer_, &QTimer::timeout, this, &ProjectPage::applyTypedPath);
    connect(pathEdit_, &QLineEdit::textEdited, this, [this] { typingTimer_.start(); });
    connect(pathEdit_, &QLineEdit::editingFinished, this, &ProjectPage::applyTypedPath);
    connect(browseButton_, &QPushButton::clicked, this, &ProjectPage::browse);
    connect(dropZone_, &ProjectDropZone::pathDropped, this, [this](const QString& path) { model_.setProjectPath(path); });
    connect(recentList_, &QListWidget::itemActivated, this, [this](QListWidgetItem* item) {
        model_.setProjectPath(item->data(Qt::UserRole).toString());
    });
    connect(recentList_, &QListWidget::itemClicked, this, [this](QListWidgetItem* item) {
        model_.setProjectPath(item->data(Qt::UserRole).toString());
    });
    connect(&model_, &WizardViewModel::projectChanged, this, &ProjectPage::refresh);
    connect(&recent_, &RecentProjects::changed, this, &ProjectPage::refreshRecent);
    connect(&ui_, &UiResources::themeChanged, this, &ProjectPage::refreshIcons);

    refreshIcons();
    refreshRecent();
    refresh();
}

void ProjectPage::refreshIcons() {
    dropIcon_->setPixmap(ui_.icon(QStringLiteral("folder-open"), IconTone::Accent).pixmap(40, 40));
    browseButton_->setIcon(ui_.icon(QStringLiteral("folder-open"), IconTone::Text));
    refreshRecent();
}

void ProjectPage::applyTypedPath() {
    typingTimer_.stop();
    if (pathEdit_->text().trimmed() != model_.projectPath()) model_.setProjectPath(pathEdit_->text());
}

void ProjectPage::browse() {
    const QString start = model_.projectPath().isEmpty() ? QString() : model_.projectPath();
    const QString dir = QFileDialog::getExistingDirectory(this, tr("Select your WPILib robot project"), start);
    if (!dir.isEmpty()) model_.setProjectPath(dir);
}

void ProjectPage::refreshRecent() {
    recentList_->clear();
    for (const QString& path : recent_.paths()) {
        const QFileInfo info(path);
        auto* item = new QListWidgetItem(ui_.icon(QStringLiteral("folder-open"), IconTone::Muted),
                                         QStringLiteral("%1\n%2").arg(info.fileName(), QDir::toNativeSeparators(path)));
        item->setData(Qt::UserRole, path);
        item->setToolTip(QDir::toNativeSeparators(path));
        recentList_->addItem(item);
    }
    recentTitle_->setVisible(recentList_->count() > 0);
    recentList_->setVisible(recentList_->count() > 0);
}

void ProjectPage::refresh() {
    const ProjectInspection& in = model_.inspection();
    if (pathEdit_->text().trimmed() != in.path && (!pathEdit_->hasFocus() || model_.projectPath().isEmpty()))
        pathEdit_->setText(model_.projectPath());
    const bool blank = model_.projectPath().isEmpty();
    pathEdit_->setProperty("invalid", !blank && !in.usable());
    repolish(pathEdit_);

    if (blank) {
        banner_->clear();
        detailsCard_->hide();
        return;
    }
    if (!in.usable()) {
        banner_->setMessage(BannerKind::Error, in.notes.empty() ? tr("This is not a WPILib project.") : in.notes.front().text);
        detailsCard_->hide();
        return;
    }

    QStringList warnings;
    for (const auto& n : in.notes)
        if (n.severity != sdk::Severity::Info) warnings << n.text;
    if (warnings.isEmpty()) banner_->setMessage(BannerKind::Success, tr("WPILib project “%1” found.").arg(in.projectName));
    else banner_->setMessage(BannerKind::Warning, tr("WPILib project “%1” found, with notes:\n• %2").arg(in.projectName, warnings.join(QStringLiteral("\n• "))));

    seasonValue_->setText(in.wpilibYear > 0 ? tr("%1 (GradleRIO %2)").arg(in.wpilibYear).arg(in.gradleRioVersion.isEmpty() ? tr("version unknown") : in.gradleRioVersion)
                                            : tr("Unknown"));
    languageValue_->setText(in.language == QLatin1String("java") ? tr("Java") : in.language == QLatin1String("cpp") ? tr("C++") : tr("Unknown"));
    teamValue_->setText(in.teamNumber > 0 ? QString::number(in.teamNumber) : tr("Not set"));
    QStringList vendors;
    for (const auto& d : in.vendorDeps) vendors << (d.version.isEmpty() ? d.name : QStringLiteral("%1 %2").arg(d.name, d.version));
    vendorValue_->setText(vendors.isEmpty() ? tr("None found") : vendors.join(QStringLiteral(", ")));
    setupValue_->setText(model_.hasExistingConfig() ? tr("Already configured — finishing will update it and keep your mechanisms and device mappings.")
                                                    : tr("New — the next steps create it."));
    detailsCard_->show();
}

}  // namespace wpiemu::app
